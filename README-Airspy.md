# dump1090-mutability with Airspy support

For details on functionality/enhancements of the mutability fork of dump1090, please see README.md.

This version is licensed under the GPL (v2 or later).
See the file COPYING for details.

# Building with Airspy support (Unix variants)

To enable support for Airspy devices, you will need the airspy and sox resampler libs (in addition to the rtlsdr and usb-1.0 libraries) when building this package.

You will likely need to build and install libairspy from source... it can be found [here](https://github.com/airspy/host).

You may be able to do something like the following to get the libsoxr lib and development files:

````
$ sudo apt-get install libsoxr-dev
````

If that (or the equivalent for your distribution) doesn't work, then you will need to build and install it from the source found [here](https://sourceforge.net/projects/soxr/files/).

Once the airspy and soxr libs are installed, you can simply do this to build dump1090 with support for both RTL-SDR and Airspy devices:

````
$ make AIRSPY=1 dump1090
````

Or you can do this to build dump1090, view1090 and faup1090:

````
$ make AIRSPY=1
````

If using another approach to build this package, be sure to include preprocessor define macro HAVE_AIRSPY if you wish to include support for the Airspy.

For details on usage after building, do this:

````
$ dump1090 --help
````
