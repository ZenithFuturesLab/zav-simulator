#pragma once

#include <istream>
#include <string>
#include <vector>

namespace epic_ros {

inline bool readLkhTour(std::istream &file, int dimension, std::vector<int> &indices) {
  indices.clear();
  if (dimension < 3) return false;
  std::string line;
  while (std::getline(file, line) && line != "TOUR_SECTION") {}
  if (line != "TOUR_SECTION") return false;

  std::vector<int> tour;
  std::vector<bool> seen(dimension, false);
  tour.reserve(dimension);
  for (int id; file >> id && id != -1;) {
    if (id < 1 || id > dimension || seen[id - 1]) return false;
    tour.push_back(id - 1);
    seen[id - 1] = true;
  }
  std::string end;
  if (!(file >> end) || end != "EOF" || tour.size() != seen.size() || tour.front() != 0) return false;
  indices.swap(tour);
  return true;
}

}  // namespace epic_ros
