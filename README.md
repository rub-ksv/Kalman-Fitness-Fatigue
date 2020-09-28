# Kalman-Fitness-Fatigue
Estimate fitness and fatigue better via Kalman filtering (on top of the classical fitness-fatigue model)

Main_FF.m: 
  Main file to run parameter estimation and test

tested under Matlab2020a, requires Simulink and the Optimization Toolbox

This code is provided without any warranty, express or implied.

None of the provided example data are from real experiments.

The code is provided only to add some implementation details that may be helpful to implement or analyze aspects of
the paper "Performance Estimation using the Fitness-Fatigue Model with Kalman Filter Feedback" under 
https://content.sciendo.com/view/journals/ijcss/16/2/article-p117.xml?language=en 

But, beware... the code requires a bit of consideration: It is not guaranteed to produce stable results on new data and requires quite a bit of thinking about what reasonable values are for the search ranges for all model parameters. If you search outside these limits, you may end up with an unstable system model, which then would produce quite unrealistic results.

Also, the data that I have provided is in the right structure, but not realistic, so when the model converges on this weird data, it may not, with your actual measurements.
