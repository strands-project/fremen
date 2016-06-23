# FRONGO
## THE FREMEN-MONGODB LINK YOU'VE BEEN LOOKING FOR
### What is Frongo?

Frongo is a Fremen-based temporal extension for MongoDB,
in other words it is a tool for converting mongodb queries into temporal models using Frequency Map Enhancement (FreMEn).

### Why would I need such a thing?

The Frequency Map Enhancement (FreMEn) models the uncertainty of elementary states by a combination of periodic functions
rather than by a constant probability,this allows the integration of observations into memory-efficient temporal models. 
These models can predict future states with a given level of confidence. 

Combining the predictive power of the FreMEn models with MongoDB's support for advanced queries,
can be extremely useful to create state predictions based on the information stored in the DB.

## How does it work?

Frongo is a ROS node that can be launched running:

`rosrun frongo fremeniser.py [-h] [-yaml_defs YAML_DEFS]`

Where YAML_DEFS is a file defining the Fremen model to be created:

```YAML
- model:
    name: "Presence"
    db: "openhab"
    collection: "openhab"
    query: '{"item":"Presence"}'
    timestamp_field: "timestamp"
    data_field: "value"
    data_type: "float"
- model:
    name: "EMC"
    db: "openhab"
    collection: "openhab"
    query: '{"item":"Entry_Multi_Contact"}'
    timestamp_field: "timestamp"
    data_field: "value"
    data_type: "boolean"
    data_conf: '{"True":["OPEN"], "False":["CLOSED"]}'
```
