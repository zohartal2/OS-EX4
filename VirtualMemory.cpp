#include "VirtualMemory.h"
#include "PhysicalMemory.h"
#include "MemoryConstants.h"

// Helper function to extract bits from a virtual address at a specific level
uint64_t extractBits(uint64_t virtualAddress, int level) {
  int bitsPerLevel = OFFSET_WIDTH;
  int shift = OFFSET_WIDTH + (TABLES_DEPTH - 1 - level) * bitsPerLevel;
  uint64_t mask = (1LL << bitsPerLevel) - 1;
  return (virtualAddress >> shift) & mask;
}

// Helper function to get the page number from virtual address
uint64_t getPageNumber(uint64_t virtualAddress) {
  return virtualAddress >> OFFSET_WIDTH;
}

// Helper function to get offset from virtual address
uint64_t getOffset(uint64_t virtualAddress) {
  uint64_t mask = (1LL << OFFSET_WIDTH) - 1;
  return virtualAddress & mask;
}

// Helper function to calculate cyclical distance
uint64_t cyclicalDistance(uint64_t page1, uint64_t page2) {
  uint64_t direct = (page1 > page2) ? (page1 - page2) : (page2 - page1);
  uint64_t wraparound = NUM_PAGES - direct;
  return (direct < wraparound) ? direct : wraparound;
}

// DFS traversal to find empty table, unused frame, or page to evict
// Parameters passed by reference to avoid dynamic allocation
void dfsSearch(uint64_t frameIndex, uint64_t currentPage, int depth,
               uint64_t targetPage, uint64_t parentFrame, uint64_t parentIndex,
               uint64_t& bestFrame, int& bestType, uint64_t& bestPageToEvict,
               uint64_t& bestParentFrame, uint64_t& bestParentIndex,
               uint64_t& maxFrame, uint64_t& bestDistance) {

  if (frameIndex == 0) return; // Skip invalid frames

  // Update max frame
  if (frameIndex > maxFrame) {
    maxFrame = frameIndex;
  }

  // If we've reached a leaf (actual page, not table)
  if (depth == TABLES_DEPTH) {
    // This is a page, consider it for eviction
    uint64_t distance = cyclicalDistance(targetPage, currentPage);
    if (bestType != 1 && bestType != 2) { // No empty table or unused frame found yet
      if (bestType != 3 || distance > bestDistance) {
        bestType = 3;
        bestFrame = frameIndex;
        bestPageToEvict = currentPage;
        bestParentFrame = parentFrame;
        bestParentIndex = parentIndex;
        bestDistance = distance;
      }
    }
    return;
  }

  // This is a table, check if it's empty
  bool isEmpty = true;
  for (int i = 0; i < PAGE_SIZE; i++) {
    word_t entry;
    PMread(frameIndex * PAGE_SIZE + i, &entry);
    if (entry != 0) {
      isEmpty = false;
      // Recursively search this subtable
      uint64_t nextPage = (currentPage << OFFSET_WIDTH) | i;
      dfsSearch(entry, nextPage, depth + 1, targetPage, frameIndex, i,
                bestFrame, bestType, bestPageToEvict, bestParentFrame,
                bestParentIndex, maxFrame, bestDistance);
    }
  }

  // If table is empty and we haven't found a better option
  if (isEmpty && bestType != 1) {
    bestType = 1;
    bestFrame = frameIndex;
    bestParentFrame = parentFrame;
    bestParentIndex = parentIndex;
  }
}

// Find a frame to use (empty table, unused frame, or evict a page)
// Returns frame index, or 0 if failed
uint64_t findFrame(uint64_t targetPage, int& frameType, uint64_t& pageToEvict,
                   uint64_t& parentFrame, uint64_t& parentIndex) {
  uint64_t bestFrame = 0;
  int bestType = 0;
  uint64_t bestPageToEvict = 0;
  uint64_t bestParentFrame = 0;
  uint64_t bestParentIndex = 0;
  uint64_t maxFrame = 0;
  uint64_t bestDistance = 0;

  // Start DFS from root table (frame 0)
  dfsSearch(0, 0, 0, targetPage, 0, 0, bestFrame, bestType, bestPageToEvict,
            bestParentFrame, bestParentIndex, maxFrame, bestDistance);

  // Check if there's an unused frame
  if (maxFrame + 1 < NUM_FRAMES && bestType != 1) {
    bestType = 2;
    bestFrame = maxFrame + 1;
  }

  frameType = bestType;
  pageToEvict = bestPageToEvict;
  parentFrame = bestParentFrame;
  parentIndex = bestParentIndex;

  return bestFrame;
}

// Remove reference to a frame from its parent table
void removeReference(uint64_t parentFrame, uint64_t parentIndex) {
  if (parentFrame != 0 || parentIndex != 0) { // Don't modify root table entry
    PMwrite(parentFrame * PAGE_SIZE + parentIndex, 0);
  }
}

// Translate virtual address and handle page faults
uint64_t translateAddress(uint64_t virtualAddress) {
  uint64_t currentFrame = 0; // Start from root table
  uint64_t pageNumber = getPageNumber(virtualAddress);

  // Navigate through the page table hierarchy
  for (int level = 0; level < TABLES_DEPTH; level++) {
    uint64_t index = extractBits(virtualAddress, level);
    word_t entry;
    PMread(currentFrame * PAGE_SIZE + index, &entry);

    if (entry == 0) {
      // Page fault - need to allocate a frame
      int frameType;
      uint64_t pageToEvict;
      uint64_t parentFrame;
      uint64_t parentIndex;

      uint64_t newFrame = findFrame(pageNumber, frameType, pageToEvict,
                                    parentFrame, parentIndex);

      if (newFrame == 0) {
        return 0; // Failed to find a frame
      }

      // Handle different types of frames found
      if (frameType == 1) {
        // Empty table - remove reference from its parent
        removeReference(parentFrame, parentIndex);
      } else if (frameType == 3) {
        // Need to evict a page
        PMevict(newFrame, pageToEvict);
        removeReference(parentFrame, parentIndex);
      }

      // Clear the frame if it will contain a table (not the last level)
      if (level < TABLES_DEPTH - 1) {
        for (int i = 0; i < PAGE_SIZE; i++) {
          PMwrite(newFrame * PAGE_SIZE + i, 0);
        }
      } else {
        // This is a page, restore it from swap if needed
        PMrestore(newFrame, pageNumber);
      }

      // Update parent table to point to new frame
      PMwrite(currentFrame * PAGE_SIZE + index, newFrame);
      currentFrame = newFrame;
    } else {
      currentFrame = entry;
    }
  }

  // Now we have the frame containing our page
  uint64_t offset = getOffset(virtualAddress);
  uint64_t physicalAddress = currentFrame * PAGE_SIZE + offset;

  return physicalAddress;
}

void VMinitialize() {
  // Clear frame 0 (root table)
  for (int i = 0; i < PAGE_SIZE; i++) {
    PMwrite(i, 0);
  }
}

int VMread(uint64_t virtualAddress, word_t* value) {
  if (virtualAddress >= VIRTUAL_MEMORY_SIZE) {
    return 0; // Invalid address
  }

  uint64_t physicalAddress = translateAddress(virtualAddress);
  if (physicalAddress == 0) {
    return 0; // Translation failed
  }

  PMread(physicalAddress, value);
  return 1;
}

int VMwrite(uint64_t virtualAddress, word_t value) {
  if (virtualAddress >= VIRTUAL_MEMORY_SIZE) {
    return 0; // Invalid address
  }

  uint64_t physicalAddress = translateAddress(virtualAddress);
  if (physicalAddress == 0) {
    return 0; // Translation failed
  }

  PMwrite(physicalAddress, value);
  return 1;
}