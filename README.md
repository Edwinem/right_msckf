# RIGHT MSCKF

This implements the Right MSCKF called so cause it is the right way to implement
 it with proper Hamiltonian Quaternions. Also a nice play on words as Hamiltonian Quaternions are also in the right handed
 coordinate system hence the name.
 
Main purpose is to serve as a reference for people that want to implement a MSCKF with Hamiltonian style quaternions, and not
 have to resort to using JPL style.
 
 Most of the derivations are taken from [2] a fantastic resource for ESKF derivation with hamiltonian style
 quaternions.
 
 
 
 ### Rant
 The original MSCKF was first published in [1]. It is a fantastic piece of work, the only
  downside being that Stergios and his collaborators(Mostly University of Minnesota students/alumni) use JPL style quaternions, in contrast
  to much of the robotics, math, and physics literature. Sadly everyone that decides to implement
  a MSCKF directly pulls their equations from the original source, or one of its derivatives written
  by someone associated with Stergios. This has resulted in a large number of JPL style quaternions, which I am not a
   fan of. I very much dislike this existence of two styles/coordinate systems. I believe it brings confusion, and harms
   collaboration efforts, especially when there isn't really a benefit. For more about the existence of these two styles and their history I
   recommend [4].
 
### Other comments
 
 - Note that there is some evidence in [3] that the JPL style aka Global error performs better. 
 
### TODO
 
 
 - [ ] Finish all parts of MSCKF
 - [ ] Create pdf with derivations
 - [ ] Create Visual Inertial Simulator
 
 
 
 ## References
 
    @article{Mourikis2007a,
      author = {Mourikis, Anastasios I. and Roumeliotis, Stergios I.},
      title = {{A multi-state constraint Kalman filter for vision-aided inertial navigation}},
      year = {2007}
      }

    @techreport{JoanSola2012,
    author = {{Joan Sol{\'{a}}}},
    title = {{Quaternion Kinematics for Error-State KF}},
    year = {2012}
    }
    
    @article{Li2012,
    author = {Li, Mingyang and Mourikis, Anastasios I.},
    title = {{Improving the accuracy of EKF-based visual-inertial odometry}},
    year = {2012}
    }
    
    @article{Sommer2018,
    author = {Sommer, Hannes and Gilitschenski, Igor and Bloesch, Michael and Weiss, Stephan and Siegwart, Roland and Nieto, Juan},
    title = {{Why and how to avoid the flipped quaternion multiplication}},
    year = {2018}
    }


