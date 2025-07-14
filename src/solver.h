#pragma once
#include <vector>
#include <string>
#include "cell.h"

/**
 * Finds the shortest path from start 'S' to exit 'E' in the dungeon
 * using Breadth-First Search (BFS). This is the basic pathfinding
 * algorithm that ignores keys and doors.
 * 
 * @param dungeon 2D grid represented as vector of strings
 * @return Vector of Cell coordinates representing the path from S to E.
 *         Returns empty vector if no path exists.
 */
std::vector<Cell> bfsPath(const std::vector<std::string>& dungeon);

/**
 * Advanced pathfinding that handles keys and doors using state augmentation.
 * The state includes position (row, col) plus a bitmask of collected keys.
 * Keys are 'a'-'f' and doors are 'A'-'F'. You need key 'a' to pass door 'A', etc.
 * 
 * @param dungeon 2D grid with walls '#', open spaces ' ', start 'S', exit 'E',
 *                keys 'a'-'f', and doors 'A'-'F'
 * @return Vector of Cell coordinates representing the path from S to E.
 *         Returns empty vector if no path exists.
 */
std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon);

/**
 * Helper function to find the position of a specific character in the dungeon.
 * Useful for locating the start 'S' and exit 'E' positions.
 * 
 * @param dungeon The dungeon grid to search
 * @param target The character to find ('S' or 'E')
 * @return Cell containing the position, or Cell(-1, -1) if not found
 */
Cell findPosition(const std::vector<std::string>& dungeon, char target);

/**
 * Helper function to check if a position is valid and passable.
 * A cell is passable if it's within bounds and not a wall.
 * 
 * @param dungeon The dungeon grid
 * @param row Row coordinate to check
 * @param col Column coordinate to check
 * @return true if the position is valid and passable, false otherwise
 */
bool isPassable(const std::vector<std::string>& dungeon, int row, int col);

/**
 * Helper function to check if a door can be passed given the current key collection.
 * For door 'A', you need key 'a' (bit 0 set), for door 'B', you need key 'b' (bit 1 set), etc.
 * 
 * @param door The door character ('A'-'F')
 * @param keyMask Bitmask representing collected keys (bit 0 = 'a', bit 1 = 'b', etc.)
 * @return true if the door can be passed, false otherwise
 */
bool canPassDoor(char door, int keyMask);

/**
 * Helper function to update the key collection when stepping on a key.
 * For key 'a', set bit 0, for key 'b', set bit 1, etc.
 * 
 * @param key The key character ('a'-'f')
 * @param keyMask Current bitmask of collected keys
 * @return Updated bitmask with the new key added
 */
int collectKey(char key, int keyMask);

// Uncomment the line below if you want to implement the optional intermediate challenge
#define IMPLEMENT_OPTIONAL_FUNCTIONS

#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
/**
 * OPTIONAL INTERMEDIATE CHALLENGE (easier than full key-door BFS):
 * Implement a function that just counts how many keys are reachable from start,
 * ignoring doors completely. This helps students practice the bitmask concept
 * without the complexity of door logic.
 * 
 * TO ENABLE THIS CHALLENGE:
 * 1. Uncomment the #define IMPLEMENT_OPTIONAL_FUNCTIONS line above
 * 2. Implement this function in solver.cpp
 * 3. The main program will automatically test it
 * 
 * @param dungeon The dungeon grid
 * @return Number of unique keys reachable from start position
 */
int countReachableKeys(const std::vector<std::string>& dungeon);
#endif
