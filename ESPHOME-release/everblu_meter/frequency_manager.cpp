/**
 * @file frequency_manager.cpp
 * @brief Implementation of frequency management and calibration
 */

#include "frequency_manager.h"
#include "logging.h"
#include "storage_abstraction.h"
#if defined(ESP32)
#include <esp_task_wdt.h>
#endif

// Static member initialization
float FrequencyManager::s_baseFrequency = 0.0;
float FrequencyManager::s_storedOffset = 0.0;
bool FrequencyManager::s_autoScanEnabled = true;
int FrequencyManager::s_adaptiveThreshold = 10;
int FrequencyManager::s_successfulReadsCount = 0;
float FrequencyManager::s_cumulativeFreqError = 0.0;
FrequencyManager::ScanStrategy FrequencyManager::s_scanStrategy = FrequencyManager::ScanStrategy::RSSI_ONLY;
int FrequencyManager::s_scanConfirmationReads = 1;

// Callback pointers (must be set before use)
RadioInitCallback FrequencyManager::s_radioInitCallback = nullptr;
MeterReadCallback FrequencyManager::s_meterReadCallback = nullptr;

// Cross-platform watchdog helper
void FrequencyManager::feedWatchdog()
{
#if defined(ESP8266)
    ESP.wdtFeed();
#elif defined(ESP32)
    esp_task_wdt_reset();
    yield();
#endif
}

// Validate that required callbacks are set
bool FrequencyManager::validateCallbacks()
{
    if (!s_radioInitCallback)
    {
        Serial.println("[ERROR] Radio init callback not set. Call setRadioInitCallback() first!");
        return false;
    }
    if (!s_meterReadCallback)
    {
        Serial.println("[ERROR] Meter read callback not set. Call setMeterReadCallback() first!");
        return false;
    }
    return true;
}

// Callback setters
void FrequencyManager::setRadioInitCallback(RadioInitCallback callback)
{
    s_radioInitCallback = callback;
    Serial.println("[FREQ] FrequencyManager: Radio init callback registered");
}

void FrequencyManager::setMeterReadCallback(MeterReadCallback callback)
{
    s_meterReadCallback = callback;
    Serial.println("[FREQ] FrequencyManager: Meter read callback registered");
}

float FrequencyManager::begin(float baseFrequency)
{
    s_baseFrequency = baseFrequency;

    // Validate callbacks are set
    if (!validateCallbacks())
    {
        Serial.println("[ERROR] FrequencyManager::begin() failed - callbacks not configured!");
        return 0.0;
    }

    // Initialize storage
    StorageAbstraction::begin();

    // Load stored offset
    s_storedOffset = loadFrequencyOffset();

    LOG_I("everblu_meter", "Initialized: base=%.6f MHz, offset=%.6f MHz",
          s_baseFrequency, s_storedOffset);

    return s_storedOffset;
}

float FrequencyManager::getOffset()
{
    return s_storedOffset;
}

void FrequencyManager::setOffset(float offset)
{
    s_storedOffset = offset;
}

float FrequencyManager::getBaseFrequency()
{
    return s_baseFrequency;
}

float FrequencyManager::getTunedFrequency()
{
    return s_baseFrequency + s_storedOffset;
}

void FrequencyManager::saveFrequencyOffset(float offset)
{
    StorageAbstraction::saveFloat(STORAGE_KEY, offset, STORAGE_MAGIC);
    s_storedOffset = offset;

    LOG_I("everblu_meter", "Frequency offset %.3f kHz saved", offset * 1000.0);
}

float FrequencyManager::loadFrequencyOffset()
{
    float offset = StorageAbstraction::loadFloat(STORAGE_KEY, 0.0, STORAGE_MAGIC, MIN_OFFSET, MAX_OFFSET);

    if (offset == 0.0)
    {
        LOG_I("everblu_meter", "No valid frequency offset found in storage");
    }

    return offset;
}

// --------------------------------------------------------------------------
// Scorecard scan helpers (file-local types and helpers)
// --------------------------------------------------------------------------
namespace
{
    // Maximum frequencies in a single scan (matches widest scan: ±100 kHz / 5 kHz step + slack)
    constexpr int SCAN_MAX_FREQS = 64;

    // Per-frequency scorecard accumulator used during SCORECARD scan
    struct FreqScore
    {
        float freq;
        int attempts;
        int successes;
        int rssi_sum_dbm;             // accumulated RSSI across successes (sign-preserving)
        bool history_monotonic;       // true if history non-decreasing on every success
        bool history_consistent;      // true if all successes returned identical history[]
        bool volume_consistent;       // true if all successes returned identical volume
        bool history_available;       // true if at least one success had history_available
        bool volume_in_history_range; // true if volume >= newest history value
        uint32_t first_history[13];
        int first_volume;
        int score; // 0..100, computed at end
    };

    // Score a single frequency from its accumulated FreqScore. Range 0..100.
    // Components (max contribution shown in brackets):
    //   [35] success rate            (decoded N out of attempts)
    //   [20] history monotonic       (binary)
    //   [15] history consistent      (binary, identical across attempts)
    //   [10] volume consistent       (binary, identical across attempts)
    //   [10] history_available flag  (binary)
    //   [ 5] volume in history range (binary)
    //   [ 5] RSSI normalised         (-120..-60 dBm -> 0..1)
    int compute_score(const FreqScore &s)
    {
        if (s.successes == 0 || s.attempts == 0)
            return 0;
        int score = 0;
        // success rate (rounded)
        score += (35 * s.successes + s.attempts / 2) / s.attempts;
        if (s.history_monotonic) score += 20;
        if (s.history_consistent) score += 15;
        if (s.volume_consistent) score += 10;
        if (s.history_available) score += 10;
        if (s.volume_in_history_range) score += 5;
        // RSSI normalised: mean across successes, mapped from [-120, -60] -> [0, 5]
        int mean_rssi = s.rssi_sum_dbm / s.successes;
        int rssi_norm = mean_rssi + 120; // 0..60 typical
        if (rssi_norm < 0) rssi_norm = 0;
        if (rssi_norm > 60) rssi_norm = 60;
        score += (rssi_norm * 5 + 30) / 60;
        if (score > 100) score = 100;
        return score;
    }

    bool history_is_monotonic(const uint32_t *h)
    {
        for (int i = 1; i < 13; ++i)
        {
            if (h[i] != 0 && h[i - 1] != 0 && h[i] < h[i - 1])
                return false;
        }
        return true;
    }

    bool history_arrays_equal(const uint32_t *a, const uint32_t *b)
    {
        for (int i = 0; i < 13; ++i)
            if (a[i] != b[i]) return false;
        return true;
    }
}

void FrequencyManager::performScorecardScan_(void (*statusCallback)(const char *, const char *),
                                             float scanStart, float scanEnd, float scanStep)
{
    LOG_I("everblu_meter", "[SCORECARD] Multi-read scan: %d reads/freq", s_scanConfirmationReads);
    LOG_I("everblu_meter", "[SCORECARD] Scanning from %.6f to %.6f MHz (step: %.6f MHz)",
          scanStart, scanEnd, scanStep);

    FreqScore scores[SCAN_MAX_FREQS];
    int n_freqs = 0;

    for (float freq = scanStart; freq <= scanEnd && n_freqs < SCAN_MAX_FREQS; freq += scanStep)
    {
        FreqScore &fs = scores[n_freqs++];
        fs = {};
        fs.freq = freq;
        fs.history_monotonic = true;       // assume good until proven otherwise
        fs.history_consistent = true;      // assume good until proven otherwise
        fs.volume_consistent = true;
        fs.volume_in_history_range = true; // assume good until proven otherwise
        fs.first_volume = -1;

        feedWatchdog();
        if (!s_radioInitCallback(freq))
        {
            LOG_E("everblu_meter", "[SCORECARD] Radio not responding at %.6f MHz - aborting", freq);
            return;
        }
        delay(50);

        for (int attempt = 0; attempt < s_scanConfirmationReads; ++attempt)
        {
            feedWatchdog();
            struct tmeter_data d = s_meterReadCallback();
            fs.attempts++;
            if (d.reads_counter <= 0)
            {
                // No decode this attempt
                continue;
            }
            fs.successes++;
            fs.rssi_sum_dbm += d.rssi_dbm;
            fs.history_available = fs.history_available || d.history_available;

            if (fs.successes == 1)
            {
                // Capture reference values from the first successful read
                for (int i = 0; i < 13; ++i) fs.first_history[i] = d.history[i];
                fs.first_volume = d.volume;
            }
            else
            {
                if (!history_arrays_equal(fs.first_history, d.history))
                    fs.history_consistent = false;
                if (d.volume != fs.first_volume)
                    fs.volume_consistent = false;
            }

            if (d.history_available && !history_is_monotonic(d.history))
                fs.history_monotonic = false;

            // Sanity: current volume should be >= newest non-zero history entry
            uint32_t newest_history = 0;
            for (int i = 12; i >= 0; --i)
            {
                if (d.history[i] != 0) { newest_history = d.history[i]; break; }
            }
            if (newest_history > 0 && (uint32_t)d.volume < newest_history)
                fs.volume_in_history_range = false;
        }

        fs.score = compute_score(fs);
        int mean_rssi = fs.successes > 0 ? fs.rssi_sum_dbm / fs.successes : -120;
        LOG_I("everblu_meter",
              "[SCORECARD] %.6f MHz: %d/%d ok, RSSI=%d, mono=%d cons=%d vol=%d hist_ok=%d vol_ok=%d -> score=%d",
              fs.freq, fs.successes, fs.attempts, mean_rssi,
              fs.history_monotonic ? 1 : 0, fs.history_consistent ? 1 : 0,
              fs.volume_consistent ? 1 : 0, fs.history_available ? 1 : 0,
              fs.volume_in_history_range ? 1 : 0, fs.score);
    }

    // Pick best score; tiebreak by closeness to midpoint of contiguous-success cluster.
    // Step 1: find contiguous-success cluster bounds (longest run of any-success freqs).
    int best_run_start = -1, best_run_len = 0;
    int cur_start = -1, cur_len = 0;
    for (int i = 0; i < n_freqs; ++i)
    {
        if (scores[i].successes > 0)
        {
            if (cur_len == 0) cur_start = i;
            cur_len++;
            if (cur_len > best_run_len) { best_run_len = cur_len; best_run_start = cur_start; }
        }
        else
        {
            cur_len = 0;
        }
    }

    int top_score = -1;
    for (int i = 0; i < n_freqs; ++i)
        if (scores[i].score > top_score) top_score = scores[i].score;

    if (top_score <= 0)
    {
        LOG_W("everblu_meter", "[SCORECARD] No frequency produced any successful read");
        if (statusCallback) statusCallback("Idle", "Scorecard scan failed - no signal");
        delay(100);
        s_radioInitCallback(s_baseFrequency + s_storedOffset);
        delay(100);
        return;
    }

    // Among frequencies within 2 points of top_score, pick the one closest to cluster midpoint.
    float midpoint_freq = best_run_start >= 0
                              ? scores[best_run_start + best_run_len / 2].freq
                              : s_baseFrequency;
    int best_idx = -1;
    float best_dist = 1e9;
    for (int i = 0; i < n_freqs; ++i)
    {
        if (scores[i].score >= top_score - 2)
        {
            float d = scores[i].freq > midpoint_freq ? scores[i].freq - midpoint_freq
                                                     : midpoint_freq - scores[i].freq;
            if (d < best_dist || (d == best_dist && best_idx >= 0 && scores[i].score > scores[best_idx].score))
            {
                best_dist = d;
                best_idx = i;
            }
        }
    }

    float bestFreq = scores[best_idx].freq;
    int bestScore = scores[best_idx].score;
    int bestRSSI = scores[best_idx].successes > 0
                       ? scores[best_idx].rssi_sum_dbm / scores[best_idx].successes
                       : -120;
    float offset = bestFreq - s_baseFrequency;

    LOG_I("everblu_meter",
          "[SCORECARD] Winner: %.6f MHz (score=%d, RSSI=%d dBm, offset=%.6f MHz, cluster midpoint=%.6f MHz)",
          bestFreq, bestScore, bestRSSI, offset, midpoint_freq);

    saveFrequencyOffset(offset);

    if (statusCallback)
    {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "Scorecard: offset %.3f kHz, score %d, RSSI %d dBm",
                 offset * 1000.0, bestScore, bestRSSI);
        statusCallback("Idle", msg);
    }

    delay(100);
    s_radioInitCallback(s_baseFrequency + s_storedOffset);
    delay(100);
    LOG_I("everblu_meter", "Radio reinitialized with new frequency: %.6f MHz", s_baseFrequency + s_storedOffset);
}

void FrequencyManager::performFrequencyScan(void (*statusCallback)(const char *, const char *))
{
    LOG_I("everblu_meter", "Starting frequency scan...");
    LOG_I("everblu_meter", "[NOTE] Wi-Fi/MQTT connections may temporarily drop and reconnect. This is expected.");

    // Reset adaptive tracking so the new offset has a chance to stabilize
    resetAdaptiveTracking();

    if (statusCallback)
    {
        statusCallback("Frequency Scanning", "Performing frequency scan");
    }

    // Scan range: ±30 kHz in 5 kHz steps (±0.03 MHz in 0.005 MHz steps)
    float scanStart = s_baseFrequency - 0.03;
    float scanEnd = s_baseFrequency + 0.03;
    float scanStep = 0.005;

    if (s_scanStrategy == ScanStrategy::SCORECARD)
    {
        performScorecardScan_(statusCallback, scanStart, scanEnd, scanStep);
        return;
    }

    float bestFreq = s_baseFrequency;
    int bestRSSI = -120; // Start with very low RSSI

    LOG_I("everblu_meter", "Scanning from %.6f to %.6f MHz (step: %.6f MHz)", scanStart, scanEnd, scanStep);

    for (float freq = scanStart; freq <= scanEnd; freq += scanStep)
    {
        feedWatchdog();

        // Reinitialize radio with this frequency (via injected callback)
        s_radioInitCallback(freq);
        delay(50); // Allow time for frequency to settle

        // Try to get meter data (via injected callback)
        struct tmeter_data test_data = s_meterReadCallback();

        LOG_I("everblu_meter", "Freq %.6f MHz: RSSI=%d dBm, reads=%d", freq, test_data.rssi_dbm, test_data.reads_counter);

        if (test_data.rssi_dbm > bestRSSI && test_data.reads_counter > 0)
        {
            bestRSSI = test_data.rssi_dbm;
            bestFreq = freq;
            LOG_I("everblu_meter", "Better signal at %.6f MHz: RSSI=%d dBm", freq, test_data.rssi_dbm);
        }
    }

    // Calculate and save the offset
    float offset = bestFreq - s_baseFrequency;
    LOG_I("everblu_meter", "Frequency scan complete. Best frequency: %.6f MHz (offset: %.6f MHz, RSSI: %d dBm)",
          bestFreq, offset, bestRSSI);

    if (bestRSSI > -120)
    {
        // Found a signal - save offset
        saveFrequencyOffset(offset);

        if (statusCallback)
        {
            char msg[128];
            snprintf(msg, sizeof(msg), "Scan complete: offset %.3f kHz, RSSI %d dBm", offset * 1000.0, bestRSSI);
            statusCallback("Idle", msg);
        }

        // Reinitialize with the tuned frequency to ensure consistency with normal operation
        // This ensures the radio uses the same frequency calculation as getTunedFrequency()
        // Wait a moment before reinitializing to let radio settle after scan
        delay(100);
        s_radioInitCallback(s_baseFrequency + s_storedOffset);
        // Additional stabilization delay after reinitialization
        delay(100);
        LOG_I("everblu_meter", "Radio reinitialized with new frequency: %.6f MHz", s_baseFrequency + s_storedOffset);
    }
    else
    {
        Serial.println("[FREQ] Frequency scan failed - no valid signal found");

        if (statusCallback)
        {
            statusCallback("Idle", "Frequency scan failed - no signal");
        }

        // Restore original frequency with stabilization delay
        delay(100);
        s_radioInitCallback(s_baseFrequency + s_storedOffset);
        delay(100);
        LOG_I("everblu_meter", "Radio restored to frequency: %.6f MHz", s_baseFrequency + s_storedOffset);
    }
}

void FrequencyManager::performWideInitialScan(void (*statusCallback)(const char *, const char *))
{
    Serial.println("[FREQ] Performing wide initial scan (first boot - no saved offset)...");

    // Reset adaptive tracking so the new offset has a chance to stabilize
    resetAdaptiveTracking();

    if (statusCallback)
    {
        statusCallback("Initial Frequency Scan", "First boot: scanning for meter frequency");
    }

    float bestFreq = s_baseFrequency;
    int bestRSSI = -120;

    // Wide scan: ±100 kHz in 10 kHz steps for faster initial discovery
    float scanStart = s_baseFrequency - 0.10;
    float scanEnd = s_baseFrequency + 0.10;
    float scanStep = 0.010;

    LOG_I("everblu_meter", "Wide scan from %.6f to %.6f MHz (step: %.6f MHz)", scanStart, scanEnd, scanStep);
    LOG_I("everblu_meter", "This may take 1-2 minutes on first boot...");

    for (float freq = scanStart; freq <= scanEnd; freq += scanStep)
    {
        feedWatchdog();

        // Check if radio initialization succeeds
        if (!s_radioInitCallback(freq))
        {
            LOG_E("everblu_meter", "Radio not responding - skipping wide initial scan");
            LOG_E("everblu_meter", "Check: 1) Wiring connections 2) 3.3V power supply 3) SPI pins");

            if (statusCallback)
            {
                statusCallback("Error", "[ERROR] Radio not responding - cannot scan");
            }

            return; // Exit scan
        }

        delay(100); // Longer delay for frequency to settle during wide scan

        struct tmeter_data test_data = s_meterReadCallback();

        if (test_data.rssi_dbm > bestRSSI && test_data.reads_counter > 0)
        {
            bestRSSI = test_data.rssi_dbm;
            bestFreq = freq;
            LOG_I("everblu_meter", "Found signal at %.6f MHz: RSSI=%d dBm", freq, test_data.rssi_dbm);
        }
    }

    if (bestRSSI > -120)
    {
        // Found a signal - perform fine scan around it
        LOG_I("everblu_meter", "Performing fine scan around %.6f MHz...", bestFreq);
        float fineStart = bestFreq - 0.015;
        float fineEnd = bestFreq + 0.015;
        float fineStep = 0.003;
        int fineBestRSSI = bestRSSI;
        float fineBestFreq = bestFreq;

        for (float freq = fineStart; freq <= fineEnd; freq += fineStep)
        {
            feedWatchdog();

            if (!s_radioInitCallback(freq))
            {
                LOG_E("everblu_meter", "Radio not responding during fine scan - aborting");
                break;
            }

            delay(50);

            struct tmeter_data test_data = s_meterReadCallback();

            if (test_data.rssi_dbm > fineBestRSSI && test_data.reads_counter > 0)
            {
                fineBestRSSI = test_data.rssi_dbm;
                fineBestFreq = freq;
                LOG_I("everblu_meter", "Refined signal at %.6f MHz: RSSI=%d dBm", freq, test_data.rssi_dbm);
            }
        }

        bestFreq = fineBestFreq;
        bestRSSI = fineBestRSSI;

        float offset = bestFreq - s_baseFrequency;
        LOG_I("everblu_meter", "Initial scan complete! Best frequency: %.6f MHz (offset: %.6f MHz, RSSI: %d dBm)",
              bestFreq, offset, bestRSSI);

        saveFrequencyOffset(offset);

        if (statusCallback)
        {
            char msg[128];
            snprintf(msg, sizeof(msg), "Initial scan complete: offset %.3f kHz", offset * 1000.0);
            statusCallback("Idle", msg);
        }

        // Reinitialize with stabilization delays
        delay(100);
        s_radioInitCallback(s_baseFrequency + s_storedOffset);
        delay(100);
        LOG_I("everblu_meter", "Radio reinitialized with new frequency: %.6f MHz", s_baseFrequency + s_storedOffset);
    }
    else
    {
        Serial.println("[FREQ] Wide scan failed - no meter signal found!");
        Serial.println("[FREQ] Please check:");
        Serial.println("[FREQ]  1. Meter is within range (< 50m typically)");
        Serial.println("[FREQ]  2. Antenna is connected to CC1101");
        Serial.println("[FREQ]  3. Meter serial/year are correct");
        Serial.println("[FREQ]  4. Current time is within meter's wake hours");

        if (statusCallback)
        {
            statusCallback("Idle", "Initial scan failed - check setup");
        }

        s_radioInitCallback(s_baseFrequency);
    }
}

void FrequencyManager::adaptiveFrequencyTracking(int8_t freqest)
{
    // FREQEST is a two's complement value representing frequency offset
    // Resolution is approximately Fxosc/2^14 ≈ 1.59 kHz per LSB (for 26 MHz crystal)

    // Accumulate the frequency error
    float freqErrorMHz = (float)freqest * FREQEST_TO_MHZ;
    s_cumulativeFreqError += freqErrorMHz;
    s_successfulReadsCount++;

    LOG_I("everblu_meter", "FREQEST: %d (%.4f kHz error), cumulative: %.4f kHz over %d reads",
          freqest, freqErrorMHz * 1000, s_cumulativeFreqError * 1000, s_successfulReadsCount);

    // Only adapt after N successful reads to avoid over-correcting on noise
    if (s_successfulReadsCount >= s_adaptiveThreshold)
    {
        float avgError = s_cumulativeFreqError / s_adaptiveThreshold;

        // Only adjust if average error is significant (> 2 kHz)
        if (abs(avgError * 1000) > ADAPT_MIN_ERROR_KHZ)
        {
            LOG_I("everblu_meter", "Adaptive adjustment: average error %.4f kHz over %d reads",
                  avgError * 1000, s_adaptiveThreshold);

            // Adjust the stored offset (apply 50% of the measured error to avoid over-correction)
            float adjustment = avgError * ADAPT_CORRECTION_FACTOR;
            s_storedOffset += adjustment;

            LOG_I("everblu_meter", "Adjusting frequency offset by %.3f kHz (new offset: %.3f kHz)",
                  adjustment * 1000.0, s_storedOffset * 1000.0);

            saveFrequencyOffset(s_storedOffset);

            // Reinitialize radio with adjusted frequency
            s_radioInitCallback(s_baseFrequency + s_storedOffset);
        }
        else
        {
            LOG_I("everblu_meter", "Frequency stable (avg error %.4f kHz < %.1f kHz threshold)",
                  avgError * 1000, ADAPT_MIN_ERROR_KHZ);
        }

        // Reset accumulators
        resetAdaptiveTracking();
    }
}

void FrequencyManager::resetAdaptiveTracking()
{
    s_cumulativeFreqError = 0.0;
    s_successfulReadsCount = 0;
    LOG_I("everblu_meter", "Adaptive frequency tracking reset");
}

bool FrequencyManager::shouldPerformAutoScan()
{
    return s_autoScanEnabled && (s_storedOffset == 0.0);
}

void FrequencyManager::setAutoScanEnabled(bool enabled)
{
    s_autoScanEnabled = enabled;
}

void FrequencyManager::setAdaptiveThreshold(int threshold)
{
    s_adaptiveThreshold = threshold;
}

void FrequencyManager::setScanStrategy(FrequencyManager::ScanStrategy strategy)
{
    s_scanStrategy = strategy;
}

void FrequencyManager::setScanConfirmationReads(int reads)
{
    if (reads < 1) reads = 1;
    if (reads > 10) reads = 10;
    s_scanConfirmationReads = reads;
}
