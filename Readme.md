# Surface Design by Envelopes
This project is an exploration of envelope surfaces, which are of great importance in the CNC industry.
This repository is the prototype environment described in this paper (To Be Published soon), as Unity lends itself easier to quick experimentation and vector visualization, compared to the [final product](https://github.com/tomcouperus/moving_cylinders) made in C++.

Computer numerically controlled machining has been the leading subtractive manufacturing technique for decades. 
It is used in the automotive, aeronautic, and electronic industries, among many others. Many of these industries require high precision and smoothness in the finishing stage of their products. 
This can be difficult to achieve, especially on free-form objects. 
As such, much research has been done on improving the tool motions and tool geometry required to achieve the desired smoothness and precision of free-form objects. 
Most of this research has come from the computer aided design space, and focused on approximating the tool motions and tool geometry. 

In this study, we follow up on the research by Bassegoda (2024), which instead focuses on determining the exact necessary and sufficient conditions to achieve tangent continuity between two adjacent surfaces created by separate passes of a milling tool. 
Our goal is to determine the feasibility of a novel computer-aided design (CAD) method that integrates the two main steps of the current CAD process: designing an object and then generating a tool path that manufactures it. 
As such, we define the conditions for connecting two nonadjacent surfaces via a third intermediary surface with G0-continuity. 
We visualize these conditions by expanding on the original application by Bassegoda (2024).

## Application Features
- Conical and drum-shaped tools moving through space
- Connect two envelopes with G0 or G1 continuity
- Connect two envelopes via an intermediary third with G0 continuity, in specific circumstances described in this thesis (To Be Published soon)
- Easy exploration of the various vectors involved.

## Installation
This application is made in C# with Unity.
Use [Unity](https://unity.com/download) to build and run the program.

## Related Theses

1. [C. Aranda Bassegoda: Design of continuous envelope surfaces with cylindrical tools, University of Groningen, 2024](https://fse.studenttheses.ub.rug.nl/33739/)
2. [T. Couperus: Towards Surface Design by Envelopes, University of Groningen, 2025](https://fse.studenttheses.ub.rug.nl/36627/)
