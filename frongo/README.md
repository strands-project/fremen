# FRONGO
## THE FREMEN-MONGODB LINK YOU'VE BEEN LOOKING FOR
### What is Frongo?

Frongo is a Fremen-based temporal extension for MongoDB, in other words it is a tool for converting mongodb queries into temporal models using Frequency Map Enhancement (FreMEn).

### Why would I need such a thing?

The Frequency Map Enhancement (FreMEn) models the uncertainty of elementary states by a combination of periodic functions rather than by a constant probability,this allows the integration of observations into memory-efficient temporal models. These models can predict future states with a given level of confidence. 

Combining the predictive power of the FreMEn models with MongoDB's support for advanced queries,
can be extremely useful to create state predictions based on the information stored in the DB.

## How does it work?

Frongo is a *ROS* node that can be executed running:

`rosrun frongo fremeniser.py [-h] [-yaml_defs YAML_DEFS]`

Where YAML_DEFS is a file defining the Fremen model to be created, defining the query and data type that is to be modeled, an example file could be like this:

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

You can also create your own models in runtime sending the *YAML* configuration as a string to the node via the  `/frongo/predict_models` service.


## What do all these fields mean?

Lets look at them again with the default values for each of them:


```YAML
- model:                                    # Mandatory: This is the root definition of a new model
    name: "model_1"                         # Mandatory: This is the name of the model you *MUST* define this value
    db: "message_store"                     # Optional: This is the the db name for the query
    collection: "message_store"             # Optional: This is the the collection name for the query
    query: '{}'                             # Optional: This is the query to be made
    timestamp_field: "_meta.inserted_at"    # Optional: This is the field where the timestamp is within the entry
    timestamp_type: 'datetime'              # Optional: This is the timestamp type it can either be 'datetime' for datetime objects or 'int' for epoch values
    data_field: "data"                      # Optional: This is the field where the value to be modeled is found
    data_type: "boolean"                    # Optional: This is the type of the data to be modeled you can have 'boolean' values for true or false or 'float' for values between 0 and 1
    data_conf: '{*}'                        # Optional: This is a JSON string where you can define some preprocessing for the values * See next section for understanding its set-up
```

## But, how do I define the `data_conf` field?

This a JSON string where you can define some preprocessing for the values, for *boolean* models such definition looks like this:

```JSON
{
    "True":["OPEN", "TRUE", 1], 
    "False":["CLOSED", "FALSE", 0]
}
```
Where *True* is a list of the values that should be considered as **1** in your fremen model, and *False* is a list of the values that should be considered as **0**

