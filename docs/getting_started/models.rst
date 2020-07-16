.. _chapter-models:

Models
======

From the getting started chapter, we know about the :py:class:`kineverse.model.articulation_model.ArticulationModel` class as the basic implementation of Kineverse's articulation model.
There are two more models which will be introduced in this section. Both are extensions of the first model that add a few more specialized features to it.

EventModel
----------

The first extension that we take a look at is :py:class:`kineverse.model.event_model.EventModel`. As the name suggests, it adds event listening and emission to the model. 
Specifically, it is possible to listen for changes that are being made to any part of the model, be that the data, the constraints, or the operations used for model construction.

In the following we will learn how to listen for changes in these three different aspects.

Listening for Changes in Data
-----------------------------

Let us start with the most obvious thing to listen for: Changes in the data of the model.

.. code-block:: python
    
    from kineverse.model.event_model import EventModel

    # Some callback function
    def on_changed(new_data):
        print('There was a change! Data is now: {}'.format(new_data))

    # Setting up the model
    km = EventModel()

    km.register_on_model_changed('my_var', on_changed)

    km.set_data('my_var', 5)
    km.dispatch_events()

    # > 'There was a change! Data is now: 5'


What have we learned from this snippet? First off, we learned that callbacks can be registered for a path in the model and that the callback is a function getting passed a single argument: The updated data.
We have also learned, that we can register callbacks for paths that do not yet exist in the model. Lastly, we notice that we have to actively dispatch the events that have been accumulated by the model. 

This last observation is an important one to make. It tells us, that the model will accumulated all events between two calls of *dispatch_events*, no matter if that is semantically sensible or not.

Before we look at listenting for constraint changes, let us take a final look at how the path we are listening for affects the events we receive.

.. code-block:: python

    # Callback functions
    def on_my_dict_changed(new_data):
        print('There was a change to my_dict!')

    def on_foo_changed(new_data):
        print('There was a change to foo!')

    def on_bar_changed(new_data):
        print('There was a change to bar!')

    # Setting up the model
    km = EventModel()

    km.set_data('my_dict', {'foo': 5, 'bar': 1})
    km.dispatch_events()

    km.register_on_model_changed('my_dict', on_my_dict_changed)
    km.register_on_model_changed('my_dict/foo', on_foo_changed)
    km.register_on_model_changed('my_dict/bar', on_bar_changed)

    km.set_data('my_dict/foo', 3)
    km.dispatch_events()
    # > 'There was a change to my_dict!'
    # > 'There was a change to foo!'

    km.set_data('my_dict/lol', 3)
    km.dispatch_events()
    # > 'There was a change to my_dict!'


The example above shows how listening to a path is interpreted by the model. A listener will always receive a callback, when any part of the tree at, or below that path is changed. 

For completeness' sake, we should note that callbacks can be removed using the :py:meth:`kineverse.model.event_model.EventModel.deregister_on_model_changed` function. It is possible to issue the removal or addition of callbacks as a part of a callback, but it should be noted, that these requests will only be processed after the event dispatch. The following example illustrates the effect this has:


.. code-block:: python

    # Setting up the model
    km = EventModel()


    # Callback functions
    def on_b_changed(new_data):
        print('There was a change to b!')

    def on_c_changed(new_data):
        print('There was a change to c!')

    def on_a_changed(new_data):
        print('There was a change to a!')
        km.deregister_on_model_changed(on_b_changed)
        km.register_on_model_changed('c', on_c_changed)

    km.set_data('a', 5)
    km.set_data('b', 3)
    km.set_data('c', 1)
    km.dispatch_events()

    km.register_on_model_changed('a', on_a_changed)
    km.register_on_model_changed('b', on_b_changed)

    km.set_data('a', 3)
    km.set_data('b', 2)
    km.set_data('c', 0)
    km.dispatch_events()
    # > 'There was a change to a!'
    # > 'There was a change to b!'

    km.set_data('b', 1)
    km.set_data('c', 1)
    km.dispatch_events()
    # > 'There was a change to c!'


In the example above, there are three paths in the model (*a, b, and c*). The callback for *a* removes the callback for *b*, and registers a callback for *c*. When all three values are modified and the events dispatched, we see that the callback for *b* still receives its event, while the newly registered one for *c* does not. This is, because these updates to the callback registry are postponed until after the dispatching of all events.
When *b* and *c* are modified again, we see that our updates have now taken effect.

Listening for Changes in Constraints
------------------------------------