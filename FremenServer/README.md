# Overview

The fremenserver package contains FreMEn-based  navigation.

# Practical 

The FreNap package contains an action server that receives six types of goals specified in the *action* string that are related to a state given by its *id*.

##The **add** action

This allows to add a sequence of observed *states* along with the *times* of ofservations to the model of the state *id*.
If the *id* is given for the first time, a new state is created and filled with the values.
The *add* action remembers the last timestamp that was provided and adds only newer observation to the model.

###input:
- **id** identification of the state that is concerned. If the **id** did not exist, a new state will be created 
- **states** a sequence of 0 and 1 observed at
- **times** which are in seconds. The length of the fields **times** and **states** has to be the same.

###output:
- **success** - success of the action.
- **message** - a detailed report or error message.

##The **predict** action
This action calculates the *probabilities* of the state *id*  for the given *times*.

###input:
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **times** at which the **probabilities** of the state should be estimated (in seconds).
- **order** of the model used for the estimation. The **order** equals to the number of periodic processes to be modeled. Setting order to 0 results in a static model, i.e. with **probabilities** constant in time.

###output:
- **success** - success of the action.
- **message** - a detailed report or error message.
- **probabilities**: list of predicted probabilities of the state **id** at given **times**.

##The **entropy** action 
This action calculates the *entropy* of the state *id*  for the given *times*.

###input:
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **times** at which the **entropies** of the state should be estimated (in seconds).
- **order** of the model used for the estimation. The **order** equals to the number of periodic processes to be modeled. Setting order to 0 results in a static model, so that the **entropies** will be constant in time.

###output:
- **success** - success of the action.
- **message** - a detailed report or error message.
- **entropies**: list of predicted entropies of the state **id** at given **times**.

###The **evaluate** action 
The evaluate action is meant to support decisions what model order to use for probability and entropy predictions.
It allows to calculate the prediction error of the various orders models with differen of a given (by *id*) state .
The user provides a sequence of observed *states* and *times* of observations and a maximal *order* to be evaluated.
The server performs predictions for the given *times* and *orders*, compares those with the provided *states* and calculates the percetage of unsuccessful predictions for model orders from 0 to *order*.

###3input:
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
- **states** is a sequence of zeros and one indicating the states at  
- **times** which are in seconds. The length of the fields **times** and **states** has to be the same.
- **order** is the maximal model order to be evaluated. 

####output:
- **success** contains the best performing model order.
- **message** contains a detailed report or an error message.
- **errors** is an array of prediction errors for model orders from 0 to **order**.


###The 'delete' action 
Deletes a state with a given **id** from the state collection held in the server.
####Inputs
- **id** identification of the state that is concerned. If the **id** did not exist, an error is reported.
####Output
- **success** contains the number of states in the collection before the action was called. 
- **message** contains a detailed report or an error message.

###The 'update' action 

Reserved for future use when FreMEn is fused with Gaussian Mixture Models.
