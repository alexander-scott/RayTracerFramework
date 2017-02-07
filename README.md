RayTracerFramework
=======

Overview
---------------
This project involves creating a basic ray-tracing framework that renders a video, optimising it and then porting it to PS4. 

This project only contains the C++ version not the PS4 version however. It features many bits of functionality including:

- A basic ray-tracing algorithm that renders spheres and/or cubes

- A solar system scene displaying planets rotating around a sun

- Rendering of each frame into a .ppm file

- Automatically executing FFMPEG to stitch together all the .ppm files to create a video

- The ray-tracing is multi-threaded

- The ray-tracing is optimised via screen-space subdivision (splitting the screen up into smaller sections and computing them in parallel)

- A basic configuration file to change execution paths

Execution speeds at different optimisation levels
---------------
Rendering a Solar System (1920x1080, 60fps, 5 seconds) - Release mode - Intel® Core™ i5-4690K CPU @ 3.50 GHz

Single threaded results (seconds)
Od - 460.163
O1 - 156.972
O2 - 136.414

Multi-threaded results (seconds)
Od - 250.702
O1 - 51.0268
O2 - 46.7039

Multi-threaded extra optimisation results (seconds)
O2 - 41.3463