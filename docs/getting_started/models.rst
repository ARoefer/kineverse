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

Next we are going to look at listening for changes in constraints. The first difference to listening for changes in data is that it is not possible to listen for changes in specific constraints. While constraints are uniquely identified by an Id, the general philosophy of Kineverse is that these Ids are meta information and should not be used when processing constraints. Instead, change listeners are set up by providing a set of symbols that a constraint might affect.
An example in code:

.. code-block:: python
    
    from kineverse.model.articulation_model import Constraint

    # Setting up the model
    km = EventModel()

    # Callback functions
    def on_ab_constraint_changed(c_id, c):
        if c is not None:
            print('Constraint "{}" affecting a and b was added.'.format(c_id))
        else:
            print('Constraint "{}" affecting a and b was removed.'.format(c_id))

    def on_c_constraint_changed(c_id, c):
        if c is not None:
            print('Constraint "{}" affecting c was added.'.format(c_id))
        else:
            print('Constraint "{}" affecting c was removed.'.format(c_id))

    a, b, c = [gm.Position(x) for x in 'abc']
    km.register_on_constraints_changed({a, b}, on_ab_constraint_changed)
    km.register_on_constraints_changed({c}, on_c_constraint_changed)

    km.add_constraint('constrain ab',  Constraint(-1, 1, a * b))
    km.add_constraint('constrain c',   Constraint(-1, 1, c))
    km.add_constraint('constrain abc', Constraint(-1, 1, a + b + c))
    km.dispatch_events()
    # > 'Constraint "constrain ab" affecting a and b was added.'
    # > 'Constraint "constrain c" affecting c was added.'
    # > 'Constraint "constrain abc" affecting a and b was added.'
    # > 'Constraint "constrain abc" affecting c was added.'

    km.remove_constraint('constrain ab')
    km.dispatch_events()
    # > 'Constraint "constrain ab" affecting a and b was removed.'


In the example we see how to set up listeners for different sets of symbols. We see that the callbacks are expected to be functions accepting two arguments, the first of which is the Id of the constraint, the second being the constraint itself. 
From the example we can also read that the callbacks are called for both additions and removals of constraints. In the case of a removal *None* gets passed to the callback instead of a constraint.

That was already everything there is to know on listening for constraints. Lastly, it should be noted again that callbacks can be removed using the :py:meth:`kineverse.model.event_model.EventModel.deregister_on_constraints_changed` function. As with the removal of data-callbacks the removal is postponed until after the dispatch, if events are currently being dispatched.


Listening for Changes in Operations
-----------------------------------

Listening for changes to the operations used to construct a model, differs somewhat from the previous callback registries. While the previous two methods were very much focused on using the model and thus aimed at monitoring its contents, this method is aimed at monitoring the construction of the model -- not its contents.

As described in :ref:`chapter-operations`, operations are identified using a unique Id. The callbacks are registered by specifying Ids to monitor. Instead of giving a set of Ids, a single regular expression object is passed. This is meant to facilitate an encoding of a certain kind of construction logic in the building of the model.
The default function for loading URDF models (:py:meth:`kineverse.urdf_operations.load_urdf`) uses the following convention for naming operations:

 - *create <LINK_PATH>* for operations first creating a link.
 - *connect <PARENT_PATH> <CHILD_PATH>* for operations which instantiate a joint.

Given this convention, we can register a callback that gets called whenever something gets attached to our robot's torso. Assuming we know that the torso is named *torso_link*, we can use the regular expression "connect my_robot/links/torso_link \*\\w" to identify operations that attach links to *torso_link*. The following code example illustrates this.


.. code-block:: python

    from kineverse.model.frames import Frame
    from kineverse.operations.basic_operations import CreateSingleValue, \
                                                      CreateComplexObject

    # Building the model
    km = EventModel()

    torso_frame = Frame('base')
    head_frame  = Frame('torso')

    pitch = gm.Position('pitch')
    yaw   = gm.Position('yaw')
    head_fk = gm.dot(gm.translation(0, 0, 0.3), gm.rotation3_rpy(0, pitch, yaw))

    op_create_torso = CreateComplexObject(Path('torso'), torso_frame)
    op_create_head  = CreateComplexObject(Path('head'), head_frame)
    
    