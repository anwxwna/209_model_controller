
## For choosing the controller 
Type 2,0 which is a car-like robot 
```
task_details = [[1 1 1]',[0 0 0]',[10 0 0]']
task_type = 'point'
type_model = '(2,0)' % received from the model generation unit
choose_controller(type_model,task_type,task_details)
```

## To run the component classify 
The WMR_ files have the built cars with the transformation matrix data for each wheel and actuator. The classify_components files is called within the COMPUTE section of the WMR_ file to get the required classification. This can then be used to choose the appropriate controller.
