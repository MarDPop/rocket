# rocket
rocket program 

## Description

A suite of tools to create custom model rockets

## Tools

- 6DOF
- Solid Motor Modeling
- CFD <-- coming soon
- Trade Studies <-- coming soon
- Controller Design <-- coming soon
- Launch Safety Analysis <-- coming soon

## Version History

** 0.0.1 ** initial version 
Features:
- initial GUI version with cesium. Can load and animate trajectory 
- prototype 6DoF. Simple physics models. 
- initial file saving
- initial controller and sensor modeling
More on physics
- solid rocket modeling including saint Roberts burn rate and ambient pressure compensation
- aerodynamics currently based on basic aero and stability coefficients, but capability to load aero table (coef based on mach, alpha, beta, and Reynolds, fin deflections)
- gravity and atmosphere model scales with local ground conditions, generally valid to 40km
- center of mass and moment of inertia updates with thruster burn
- huen step integration currently used but, rk23, rk4 available. 

## Directories

src/ - source files for the single stage rocket 6DOF model.

