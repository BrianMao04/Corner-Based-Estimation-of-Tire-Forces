# Corner Based Estimation of Tire Forces
Replication of results from the paper: <br />
Ehsan Hashemi, Mohammad Pirani, Amir Khajepour, Alireza Kasaiezadeh, Shih-Ken Chen, Bakhtiar Litkouhi, Corner-based estimation of tire forces and vehicle velocities robust to road conditions, Control Engineering Practice, Volume 61, 2017, Pages 28-40, ISSN 0967-0661, https://doi.org/10.1016/j.conengprac.2017.01.009.

# Instructions
Run Lateral_Force_Estimator.m in MATLAB to generate the figure depicted in the presentation. 

yk.m is a separate function for the measurement model.

All other files ("Fig8"...) is from the experimental data provided by Ehsan Hashemi. The execution of the program automatically opens each of these fig files, extracts the associated data, and subsequently closes the files (No action is required from the user to close each of these files). Afterwards, the final figure is generated. 

Note that all of the .fig files must be unzipped into the same folder as Lateral_Force_Estimator.m for the program to extract the data.

No additional tools/libraries are required beyond a standard installation of  MATLAB R2020a.
