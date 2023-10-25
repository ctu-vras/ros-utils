^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_transport_codecs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.4 (2023-10-25)
------------------

2.3.3 (2023-10-06)
------------------

2.3.2 (2023-10-06)
------------------

2.3.1 (2023-07-13)
------------------

2.3.0 (2023-07-12)
------------------
* Increased minimum CMake version to 3.10.2.
* Contributors: Martin Pecka

2.2.3 (2023-06-16)
------------------

2.2.2 (2023-05-15)
------------------

2.2.1 (2023-05-15)
------------------

2.2.0 (2023-04-09)
------------------
* Fixed getCompressedImageContent() for JPEGs in CompressedCodec.
* Swap the order of image and topic arguments in encode/decode to make the Python and C++ APIs consistent.
* Generalized getCompressedImageContent() to all codecs.
* Added getCompressedImageContent() to compressedDepth codec.
* Added guessAnyCompressedImageTransportFormat().
* Contributors: Martin Pecka

2.1.2 (2023-02-10)
------------------

2.1.1 (2023-02-08)
------------------

2.1.0 (2023-02-08)
------------------
* Verify that compressedDepth decoder input is large enough to avoid accessing invalid memory.
* Added basic input checking to RVL.
* Added image_transport_codecs.
* Contributors: Martin Pecka

2.0.10 (2022-11-24 17:43)
-------------------------

2.0.9 (2022-11-24 17:33)
------------------------

2.0.8 (2022-11-24 16:00)
------------------------

2.0.7 (2022-11-24 15:38)
------------------------

2.0.6 (2022-11-24 15:03)
------------------------

2.0.5 (2022-10-23)
------------------

2.0.4 (2022-10-14)
------------------

2.0.3 (2022-10-07)
------------------

2.0.2 (2022-08-29)
------------------

2.0.1 (2022-08-26)
------------------
