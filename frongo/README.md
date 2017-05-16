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

**`rosrun frongo fremeniser.py [-h] [-yaml_defs YAML_DEFS]`**

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

You can also create your own models in runtime sending the *YAML* configuration as a string to the node via the  **`/frongo/add_model_defs`** service. Alternatively you can the **add_models_from_yaml** script, which can be executed as follows:

**`rosrun frongo add_models_from_yaml.py definitions.yaml`**

Where **definitions.yaml** is a *YAML* file with the definitions of the models to be added.

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

This a JSON string where you can define some preprocessing for the values.

### Boolean models 
For *boolean* models such definition looks like this:

```JSON
{
    "True":["OPEN", "TRUE", 1], 
    "False":["CLOSED", "FALSE", 0]
}
```
Where *True* is a list of the values that should be considered as **1** in your fremen model, and *False* is a list of the values that should be considered as **0**

### Float type models

For *float* models you can define a linear normalisation to fit the values between **0** and **1** such definition looks like this:

```JSON
{
    "mim":0, 
    "max":1
}
```

Where *min* is the value that should be correspond to **0** and  *max* the value that corresponds to **1**

## Ok, I can create my own models, but, what do I do with them?

Once you have created the models you can get predictions using the service `/frongo/predict_models` which is as follows:

```YAML
string model_name       # SEND: The name of the model to be predicted
uint32[] epochs         # SEND: The list of epochs you want the prediction for
---
uint32[] epochs         # GET: The list of epochs the prediction was made for
float64[] predictions   # GET: The list of state probabilities for each epoch on the top list
```
This service uses the best order evaluated for this model.

If you want to create predictions with an specific fremen order you can use the `/frongo/predict_models_with_order` wich looks like this:

```YAML
string model_name
int32    order
uint32[] epochs
---
uint32[] epochs
float64[] predictions
```

The order field allows you to request predictions using any order.


## How do I setup my MongoDB address and port?

The MongoDB address and port can be set via rosparams:

**`rosparam set /mongodb_host <your mongodb server address>`**

**`rosparam set /mongodb_port <your mongodb server port>`**

## How do I know which models are already loaded?

You can use the **`/frongo/get_models`** service which look like this:

```YAML
---
string[] names
string[] info
```

Where **names** is the list of all the model names and **info** is a yaml string with the internal variables of each model.

## Is there anything else I should know?

YES, this is so awesome you can also get the model entropy over time using the services:

* **`/frongo/get_entropies`** For the best model order, which looks like this:

    ```
    string model_name
    uint32[] epochs
    ---
    uint32[] epochs
    float64[] predictions
    ```

* **`/frongo/get_entropies_with_order`** to specify the model order, like this:

    ```
    string model_name
    int32    order
    uint32[] epochs
    ---
    uint32[] epochs
    float64[] predictions
    ```
### Describe Frongoweb

here I will describe frongo web
    
### Examples

```Yaml
- model:
    name: "mon_navs_tsc_y3"
    model_type: "events"
    db: "tsc_y4_message_store"
    collection: "filtered_monitored_nav_events_y3"
    query: "{}"
    timestamp_field: "_meta.inserted_at"
    timestamp_type: 'datetime'
    data_type: "float"
    data_conf: '{"sampling": 1800}'
- model:
    name: "mon_navs_tsc_y4"
    model_type: "events"
    db: "tsc_y4_message_store"
    collection: "filtered_monitored_nav_events"
    query: "{}"
    timestamp_field: "_meta.inserted_at"
    timestamp_type: 'datetime'
    data_type: "float"
    data_conf: '{"sampling": 1800}'
```
