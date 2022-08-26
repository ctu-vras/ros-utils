^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf_static_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0
-----
* TF static publisher now doesn't spam if there are no transforms to publish. It also publishes only when the transforms really change.
* Fixed a bug with array parameters.
* tf_static_publisher can now reload the params either periodically or on a trigger
* Refactoring, allow dict/struct transform parameter.
* Moved tf_static_publisher here from tradr-ugv-base.
* Contributors: Martin Pecka, Tomas Petricek
