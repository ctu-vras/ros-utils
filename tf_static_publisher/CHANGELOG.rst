^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf_static_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2023-02-08)
------------------
* Change maintainer.
* Contributors: Martin Pecka

2.0.10 (2022-11-24)
-------------------

2.0.9 (2022-11-24)
------------------

2.0.8 (2022-11-24)
------------------

2.0.7 (2022-11-24)
------------------

2.0.6 (2022-11-24)
------------------

2.0.5 (2022-10-23)
------------------

2.0.4 (2022-10-14)
------------------

2.0.3 (2022-10-07)
------------------
* tf_static_publisher: Fix CMake warning on Noetic.
* Contributors: Martin Pecka

2.0.2 (2022-08-29)
------------------

2.0.1
-----

1.0.0
-----
* TF static publisher now doesn't spam if there are no transforms to publish. It also publishes only when the transforms really change.
* Fixed a bug with array parameters.
* tf_static_publisher can now reload the params either periodically or on a trigger
* Refactoring, allow dict/struct transform parameter.
* Moved tf_static_publisher here from tradr-ugv-base.
* Contributors: Martin Pecka, Tomas Petricek
