This is a "Special Edition" of Sibernetic software by Andrey Palyanov (original one, for C. elegans simulation, is available here: https://github.com/openworm/sibernetic) aimed at simulation of "virtual tadpole" - X. laevis tadpole 3D biomechanical body model able to swim in water simulated by prediction-correction incompressible smoothed particle hydrodynamics (PCISPH).

Source code was compiled with "Microsoft Visual Studio Community 2019" and used under Windows 7 and Windows 10.
Also it is necessary to install OpenCL drivers to allow GPU calculations, without them it will work hopelessly slow.
Even at Nvidia GTX 980 Ti the program requires about 24 hours of calculations to simulate 1 s of tadpole swimming (with about 2 millions of total particles in simulation), because numerical integration time step shouldnt' be larger than 1e-5 s (otherwise it will be unstable).
