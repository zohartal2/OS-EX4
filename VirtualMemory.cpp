#include "VirtualMemory.h"
#include "PhysicalMemory.h"
#include "MemoryConstants.h"

#include <cstdio>

#define INVALID_PHYSICAL_ADDRESS ((uint64_t)(-1))

// Helper function to extract bits from a virtual address at a specific level
uint64_t extractBits(uint64_t virtualAddress, int level) {
    // First remove the offset bits to get the page number
    uint64_t pageNumber = virtualAddress >> OFFSET_WIDTH;
    
    // Now extract the appropriate bits for this level
    // Level 0 gets the most significant bits, level TABLES_DEPTH-1 gets the least
    int shift = (TABLES_DEPTH - 1 - level) * OFFSET_WIDTH;
    uint64_t mask = (1LL << OFFSET_WIDTH) - 1;
    
    return (pageNumber >> shift) & mask;
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
void dfsSearch(uint64_t frameIndex, uint64_t virtualAddressBase, int depth, 
               uint64_t targetPage, uint64_t parentFrame, uint64_t parentIndex,
               uint64_t& bestFrame, int& bestType, uint64_t& bestPageToEvict,
               uint64_t& bestParentFrame, uint64_t& bestParentIndex,
               uint64_t& maxFrame, uint64_t& bestDistance) {
    
    if (frameIndex == 0 || frameIndex >= NUM_FRAMES) return;
    
    // Update max frame
    if (frameIndex > maxFrame) {
        maxFrame = frameIndex;
    }
    
    // If we've reached a leaf (actual page, not table)
    if (depth == TABLES_DEPTH) {
        // Calculate the actual page number for this leaf
        uint64_t currentPage = virtualAddressBase >> OFFSET_WIDTH;
        
        // Don't evict the page we're trying to bring in
        if (currentPage != targetPage) {
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
        }
        return;
    }
    
    // This is a table, check if it's empty
    bool isEmpty = true;
    bool isInTargetPath = false;
    
    // Check if this table is in the path to our target page
    uint64_t targetBits = extractBits(targetPage << OFFSET_WIDTH, depth);
    
    for (int i = 0; i < PAGE_SIZE; i++) {
        word_t entry;
        PMread(frameIndex * PAGE_SIZE + i, &entry);
        if (entry != 0) {
            isEmpty = false;
            
            // Calculate the virtual address for this subtree
            uint64_t shift = (TABLES_DEPTH - 1 - depth) * OFFSET_WIDTH;
            uint64_t nextVirtualBase = virtualAddressBase | ((uint64_t)i << shift);
            
            // Check if this entry is in the path to our target
            if (i == targetBits) {
                isInTargetPath = true;
            }
            
            // Recursively search this subtable
            dfsSearch(entry, nextVirtualBase, depth + 1, targetPage, frameIndex, i,
                     bestFrame, bestType, bestPageToEvict, bestParentFrame, 
                     bestParentIndex, maxFrame, bestDistance);
        }
    }
    
    // If table is empty and not in the path to target page
    if (isEmpty && !isInTargetPath && bestType != 1) {
        bestType = 1;
        bestFrame = frameIndex;
        bestParentFrame = parentFrame;
        bestParentIndex = parentIndex;
    }
}

// Find a frame to use (empty table, unused frame, or evict a page)
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
    for (int i = 0; i < PAGE_SIZE; i++) {
        word_t entry;
        PMread(i, &entry);
        if (entry != 0) {
            uint64_t virtualBase = (uint64_t)i << ((TABLES_DEPTH - 1) * OFFSET_WIDTH);
            dfsSearch(entry, virtualBase, 1, targetPage, 0, i, bestFrame, bestType, 
                     bestPageToEvict, bestParentFrame, bestParentIndex, 
                     maxFrame, bestDistance);
        }
    }
    
    // Check if there's an unused frame
    if (maxFrame + 1 < NUM_FRAMES && bestType != 1) {
        bestType = 2;
        bestFrame = maxFrame + 1;
    }
    
    // Fallback
    if (bestFrame == 0 && bestType == 0) {
        if (NUM_FRAMES > 1) {
            bestType = 2;
            bestFrame = 1;
        }
    }
    
    frameType = bestType;
    pageToEvict = bestPageToEvict;
    parentFrame = bestParentFrame;
    parentIndex = bestParentIndex;
    
    return bestFrame;
}

// Remove reference to a frame from its parent table
void removeReference(uint64_t parentFrame, uint64_t parentIndex) {
    // Only remove reference if it's not pointing to an invalid location
    // and not trying to modify the root table inappropriately
    if (parentFrame < NUM_FRAMES && parentIndex < PAGE_SIZE) {
        PMwrite(parentFrame * PAGE_SIZE + parentIndex, 0);
    }
}
// Fixed frame handling in translateAddress
uint64_t translateAddress(uint64_t virtualAddress) {
    uint64_t currentFrame = 0;
    uint64_t pageNumber = getPageNumber(virtualAddress);
    
    for (int level = 0; level < TABLES_DEPTH; level++) {
        uint64_t index = extractBits(virtualAddress, level);
        uint64_t tableAddress = currentFrame * PAGE_SIZE + index;
        
        if (tableAddress >= RAM_SIZE) {
            return INVALID_PHYSICAL_ADDRESS;
        }
        
        word_t entry;
        PMread(tableAddress, &entry);
        
        if (entry == 0) {
            int frameType;
            uint64_t pageToEvict;
            uint64_t parentFrame;
            uint64_t parentIndex;
            
            uint64_t newFrame = findFrame(pageNumber, frameType, pageToEvict, 
                                        parentFrame, parentIndex);
            
            if (newFrame == 0 || newFrame >= NUM_FRAMES) {
                return 0;
            }
            
            // Handle frame types correctly
            if (frameType == 1) {
                // Empty table - remove reference from its parent
                removeReference(parentFrame, parentIndex);
            } else if (frameType == 3) {
                // Need to evict a page - remove reference first, then evict
                // printf("Evicting page %llu from frame %llu\n", pageToEvict, bestFrame);
                removeReference(parentFrame, parentIndex);
                PMevict(newFrame, pageToEvict);
            }
            // frameType == 2 (unused frame) needs no special handling
            
            // Clear the frame if it will contain a table
            if (level < TABLES_DEPTH - 1) {
                for (int i = 0; i < PAGE_SIZE; i++) {
                    PMwrite(newFrame * PAGE_SIZE + i, 0);
                }
            } else {
                // This is a page, restore it
                PMrestore(newFrame, pageNumber);
            }
            
            PMwrite(tableAddress, newFrame);
            currentFrame = newFrame;
        } else {
            currentFrame = entry;
        }
    }
    
    uint64_t offset = getOffset(virtualAddress);
    uint64_t physicalAddress = currentFrame * PAGE_SIZE + offset;
    
    if (physicalAddress >= RAM_SIZE) {
        return INVALID_PHYSICAL_ADDRESS;
    }
    
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
    if (physicalAddress == INVALID_PHYSICAL_ADDRESS) {
          printf("VMread: reading from VA %llu (PA %llu) -> %d\n", virtualAddress, physicalAddress, *value);
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
    if (physicalAddress == INVALID_PHYSICAL_ADDRESS) {
        printf("VMwrite: writing %d to VA %llu (PA %llu)\n", value, virtualAddress, physicalAddress);
        return 0; // Translation failed
    }
    
    PMwrite(physicalAddress, value);
    return 1;
}