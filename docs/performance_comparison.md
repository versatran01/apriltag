# Performance comparison of detectors

This test only covers CPU speed, not the quality of the detector!

## Environment

### Hardware

- CPU: Intel i9-9900K @3.6GHz (boost to 4.8GHz), 8 cores / 16 threads
- Memory: 64GB

### Software

- OS: Ubuntu 18.04
- OpenCV: version 3.2
- ROS: melodic
- Tags: 36h11, with clean borders and border size = 1

### Images

- 1280x1024 mono chrome (OM Python 3000, PointGrey).
- number of frames: 300
- source:  directly read from bag file.
- cameras: 3 synchronized camera streams.

## Setup
Detector: 3 detectors running in parallel, one for each image, using
OMP directives. This is how the innermost loop looks:

     profiler_.reset(); // start the profiler
     #pragma omp parallel for
     for (int i = 0; i < (int)grey.size(); i++) {
        allTags[i] = detectors_[i]->Detect(grey[i]);
     }
     profiler_.record("detect", grey.size()); // stop the profiler

## Results

- UMich1:    31ms per image, running at 540% CPU
- UMich3:    16ms per image, running at 710% CPU
- Mit:       14ms per image, running at 760% CPU

So to detect all 3 frames with Mit takes 14x3 = 42msec.

## Effect of various parameters

Some more experiments with the parameters (only for UMich3):

- nthreads:
  This was running with the default nthreads, which I believe ==1 for
  UMich3. Bumping it to nthreads=3 gives 12ms/image, running at 1000% CPU.

- decimate:
  Decimating by factor of 2, UMich3 drops to 5ms per image, but also
  the number of detected tags drops by 20%.

- sigma:
  (blurring is a bad idea, the blurring step alone takes order of 10ms)
   - sigma = 0.5: 18.5ms
   - sigma = 1.0: 17.8ms
   - sigma = 2.0: 17.2ms

## UMich detector performance analysis

### UMich1
Here the performance analysis output of the original UMich library,
nthreads = 1. No decimation or blurring.

    0                             init        0.000000 ms        0.000000 ms
    1                       blur/sharp        0.000000 ms        0.000000 ms
    2                        threshold        1.662000 ms        1.662000 ms
    3                        unionfind       13.259000 ms       14.921000 ms
    4                    make clusters       16.687000 ms       31.608000 ms
    5            fit quads to clusters       45.906000 ms       77.514000 ms
    6                            quads        0.721000 ms       78.235000 ms
    7                decode+refinement        3.132000 ms       81.367000 ms
    8                        reconcile        0.000000 ms       81.367000 ms
    9                     debug output        0.000000 ms       81.367000 ms
    10                         cleanup        0.005000 ms       81.372000 ms

### UMich3

Same, but for UMich3

    0                             init        0.000000 ms        0.000000 ms
    1                       blur/sharp       10.595000 ms       10.595000 ms
    2                        threshold        1.530000 ms       12.125000 ms
    3                        unionfind        7.259000 ms       19.384000 ms
    4                    make clusters       12.465000 ms       31.849000 ms
    5            fit quads to clusters       17.379000 ms       49.228000 ms
    6                            quads        0.111000 ms       49.339000 ms
    7                decode+refinement        0.667000 ms       50.006000 ms
    8                        reconcile        0.000000 ms       50.006000 ms
    9                     debug output        0.000000 ms       50.006000 ms
    10                         cleanup        0.002000 ms       50.008000 ms
