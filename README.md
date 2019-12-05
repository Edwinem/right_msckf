# RIGHT MSCKF

This implements the Right MSCKF called so cause it is the right way to implement
 it with proper Hamiltonian Quaternions. Also a nice play on words as Hamiltonian Quaternions are also in the right handed
 coordinate system hence the name.
 
 It is to serve as a refenrnce for people that want to implement a MSCKF with Hamiltonian style quaternions, and not
 have to resort to using JPL style.
 
 
 
 ### Rant
 The original MSCKF was first published in [1]. It is a fantastic piece of work, the only
  downside being that Stergios and his collaborators(Mostly University of Minnesota students/alumni) use JPL style quaternions, in contrast
  to much of the robotics, math, and physics literature. Sadly everyone that decides to implement
  a MSCKF directly pulls their equations from the original source, or one of its derivatives written
  by someone associated with Stergios. This has resulted in a large number of JPL style quaternions, which I am not a
   fan of. I very much dislike this existence of two styles/coordinate systems. I believe it brings confusion, and harms
   collaboration efforts, especially when there isn't really a benefit. For more about the existence of these two styles and their history I
   recommend XXX.
 
### Other comments
 
 - Note that there is some evidence in XXX that the JPL style aka Global error performs better. 
 
 
 
 
 ## References
 
 [1]: @article{Mourikis2007a,
      author = {Mourikis, Anastasios I. and Roumeliotis, Stergios I.},
      title = {{A multi-state constraint Kalman filter for vision-aided inertial navigation}},
      year = {2007}
      }

 