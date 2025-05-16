This is a Sibernetic-VT software by Andrey Palyanov aimed at simulation of "virtual tadpole" - X. laevis tadpole 3D biomechanical body model able to swim in water simulated by prediction-correction incompressible smoothed particle hydrodynamics (PCISPH).
This new version also contains forceps (tweezers) that can hold a tadpole with a given degree of compression for numerical experiments.

Source code was compiled with "Microsoft Visual Studio Community 2019" and used under Windows 7 and Windows 10.
Also it is necessary to install OpenCL drivers to allow GPU calculations, without them, on CPU, it will work hopelessly slow.
At NVidia RTX 3080 Ti (10240 CUDA cores, 34.1 Tflops) the program requires about several hours of calculations to simulate 1 s of tadpole swimming (with about 2 millions of total particles in simulation), because at such scale (the tadpole is about 5 mm long) numerical integration time step shouldn't be noticeably larger than 1e-5 s (otherwise it will be unstable). Release mode instead of Debug also makes the simulation work noticeably faster.

![alt text](https://github.com/a-palyanov/sibernetic-vt/blob/main/sibernetic-vt-logo.png)
