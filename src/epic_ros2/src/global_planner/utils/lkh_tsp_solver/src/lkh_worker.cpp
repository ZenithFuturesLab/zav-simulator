#include <lkh_tsp_solver/lkh_interface.h>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  return solveTSPLKH(argv[1]);
}
