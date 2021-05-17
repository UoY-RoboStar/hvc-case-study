# hvc-case-study
Source for the HVC RoboTool model developed in the context of "Safety Assurance of a High Voltage Controller for an Industrial Robotic System" by Murray et al.

## Verification with FDR
This is the verification discussed in the paper, and the associated assertion file is `properties.assertions`.

## Verification with PRISM 
An additional verification with PRISM in RoboTool is also introduced, but this is not discussed in the paper.

The translator configuration file, and a translated PRISM model (but the order of modules and variables is manually adjusted to optimise performance of model checking in terms of memory consumption and model checking time) are under `assertions/probability_prism`.  

Assertion files are under `assertions/probability_prism`.
