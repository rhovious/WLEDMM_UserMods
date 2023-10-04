# 1 "/var/folders/nw/vdfnv6qx1dv72f_3tq2jmjsr0000gn/T/tmp0kp9zcho"
#include <Arduino.h>
# 1 "/Users/rhovious/Desktop/git/WLEDMM_UserMods/wled00/wled00.ino"
# 14 "/Users/rhovious/Desktop/git/WLEDMM_UserMods/wled00/wled00.ino"
#ifdef WLED_DEBUG_HEAP
void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name);
void esp_heap_trace_free_hook(void* ptr);
void setup();
void loop();
#line 15 "/Users/rhovious/Desktop/git/WLEDMM_UserMods/wled00/wled00.ino"
void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
  Serial.printf("*** %s failed to allocate %d bytes with ",function_name, requested_size);
  if (caps & (1 << 0)) Serial.print("Executable ");
  if (caps & (1 << 1)) Serial.print("32-Bit_Aligned ");
  if (caps & (1 << 2)) Serial.print("8-Bit_Aligned ");
  if (caps & (1 << 3)) Serial.print("DMA ");
  if (caps & (1 << 10)) Serial.print("SPI_RAM ");
  if (caps & (1 << 11)) Serial.print("Internal ");
  if (caps & (1 << 12)) Serial.print("Default ");
  if (caps & (1 << 13)) Serial.print("IRAM+unaligned ");
  if (caps & (1 << 14)) Serial.print("Retention_DMA ");
  if (caps & (1 << 15)) Serial.print("RTC_fast ");
  Serial.print("capabilities - largest free block: "+String(heap_caps_get_largest_free_block(caps)));

  size_t largest_free = heap_caps_get_largest_free_block(caps);
  size_t total_free = heap_caps_get_free_size(caps);
  float fragmentation = 100.0f;
  if ((largest_free > 1) && (total_free > largest_free))
    fragmentation = 100.f * (1.0f - (float(largest_free) / float(total_free)) );
  Serial.print("; \t available: " + String(total_free));
  Serial.print(" (frag "); Serial.print(fragmentation, 2); Serial.println("%).");

  if (!heap_caps_check_integrity_all(false)) {
    Serial.println("*** Heap CORRUPTED: "+String(heap_caps_check_integrity_all(true)));
  }
}

#if 0
void esp_heap_trace_free_hook(void* ptr)
{
  if (ptr == nullptr) {
    Serial.println("** free: attempt to free nullptr.");
  } else {
    size_t blocksize = heap_caps_get_allocated_size(ptr);
    if ((blocksize < 1) || (blocksize > 256000))
      Serial.println("**** free: bad pointer to " + String(blocksize) + "bytes.");
    else
      Serial.println("** free " + String(blocksize) + "bytes.");
  }
}
#endif
#endif

#include "wled.h"

unsigned long lastMillis = 0;
unsigned long loopCounter = 0;

void setup() {
  #ifdef WLED_DEBUG_HEAP
  esp_err_t error = heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);
  #endif
  WLED::instance().setup();
}

void loop() {

  loopCounter++;
  if (millis() - lastMillis >= 10000) {

    lastMillis = millis();
    loopCounter = 0;
  }

  WLED::instance().loop();
}