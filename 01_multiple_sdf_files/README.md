# Gazebo Tutorial: Multiple SDF Files

This tutorial demonstrates how to reference multiple SDF model files in a main SDF world, assign unique names and positions, and reuse single model descriptions for multiple entities within a Gazebo simulation.


## 1. Referencing Models in main.sdf

You can add different types of models to you world with tag `include`. It allowes to include both local models and remote models from [Gazebo Fuel](https://gazebosim.org/docs/latest/fuel_insert/). 

```xml
    <include>
        <uri>model://model_A</uri>
        <name>my_model</name>
        <pose>0 0 0 0 0 0</pose>
    </include>
    <include>
        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mine Cart Engine</uri>
        <name>mine_cart</name>
        <pose>10 0 0 0 0 0</pose>
    </include>
```

## 2. Setting Model Names and Positions

Assign a unique name to each model using `<name>`, and set its position and orientation using `<pose>`:
```xml
    <pose>x y z roll pitch yaw</pose>
```

## 3. Reusing a Model Description for Multiple Instances

You can include the same model description multiple times in your world file with different names and positions.

## 4. Running the Simulation

```bash
    cd 01_multiple_sdf_files
    gz sim main.sdf
```

## 5. Video

Video with this tutorial on [YouTube](https://youtu.be/l8bghDZGrZo)

## References

- [Gazebo SDF Documentation](https://gazebosim.org/docs/all/sdf)