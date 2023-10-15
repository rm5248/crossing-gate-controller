# Crossing Gate Controller

This is the code for a complex crossing gate controller.
The idea is that there are multiple inputs into the crontroller, and multiple
routes that a train can take through the interlocking.

This allows switches to be part of the interlocking for the train, such that
the track does not need to be continuous through the crossing gate segment.

Due to memory requirements, this will not work with an Arduino Uno.  For a
simple crossing gate controller see: https://github.com/rm5248/TrainUtils/tree/master/LCC/examples/simple-crossing-gate

## Routes

Each route through the interlocking has four inputs associated with it.
Assuming they are from left to right, they are:

1. Pre-island detection
2. Island left detection
3. Island right detection
4. Post-island detection

When the train goes from left to right, it hits sensors 1, 2, 3, 4 in order.
When it goes right to left, it hits sensors 4, 3, 2, 1.

Each route may also have up to 8 auxillary inputs associated with it, generally used for switches.

All inputs may be either hardwired to the board directly or EventIDs associated with other LCC capable boards.

## Interlocking Example

Assume we have a moderately complex interlocking that looks like the following:

```
                                   x ROAD x
                                   x      x
                                   x      x
                                   x      x
──────1──────────────────────────2┼x──────x┼3───────────────────4─────
                      \            x      x
                       \           x      x
                        \ A        x      x
                         \         x      x
                          \        x      x
──────5──────────────────────────6┼x──────x┼7───────────────────8──────
                                   x      x
                                   x      x
                                   x      x
                                   x      x
                                   x      x
                                   x      x
                                   x      x
```

If switch A is normalized, trains going from left to right will hit sensors 1, 2, 3, 4 on the top track,
or 5, 6, 7, 8 on the lower track.

If switch A is thrown, trains going from left to right will hit sensors 1, 6, 7, 8.

In order to make this interlocking work properly, we must define a total of 3 routes:

Route 1
Sensors: 1, 2, 3, 4
Switches: A(normal)

Route 2
Sensors: 1, 6, 7, 8
Switches: A(reverse)

Route 3
Sensors: 5, 6, 7, 8
Switches: A(normal)

When sensor 1 is triggered, the crontroller figures out which route the train will take by looking at the current
status of all the inputs(e.g. switches) that are in the route, and determining if this route is valid.
Once a valid route is identified, the controller keeps track of where the train is until it exits the interlocking
or the timeout expires.

Note that when defining a route, you only need to define it in one direction, the system assumes that all
routes are bi-directional.

## Timeout

If a train does not exit within 25 seconds(default), the controller will assume that the train has left the 
interlocking and will reset to the normal state.

## Outputs

When a train is first detected, outputs X, Y will be activated for the crossing gate lights in an alternating
pattern.  In addition, output Z will be activated in order to trigger an auxillary output
(e.g. lower the crossing arm).  These outputs will remain on until the train has left the interlocking, or
the timeout expires.
