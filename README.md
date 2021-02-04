# CircularInterpolation
in here we assess the efficiency of circular interpolations in cnc machines.
instead of using linear segmentation that adds a chordal error along and generates a less smooth curves,
we will be modyfing bresenham's circle algorithm, bresenham's circle has the optimum chordal error on a grid "pixels" view,
which makes it the perfect choice for a cnc machines as it's also moves in discrete steps which yields a grided workspace.
-runs on Tiva C "tm4c123gh6pm".

