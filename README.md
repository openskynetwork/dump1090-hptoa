# dump1090-hptoa

This is a fork of [Mutability's version](https://github.com/mutability/dump1090)
 of dump1090 that adds a novel method to compute high-precision Time-of-Arrival (ToA) 
 timestamps of the Mode S / ADS-B packets. The actual precision is in the order of a 
 few nanoseconds, depending on the packet strength. [Original README](README-mutability.md)

This version is licensed under the GPL (v2 or later).
See the file COPYING for details.

This new version is based on the work named ['Nanosecond-precision Time-of-Arrival Estimation for Aircraft Signals with low-cost SDR Receivers'](http://eprints.networks.imdea.org/1768/)
published in [ACM/IEEE IPSN 2018](http://ipsn.acm.org/2018/program.html) conference and made by 
[Roberto Calvo-Palomino](http://people.networks.imdea.org/~roberto_calvo/),
 [Fabio Ricciato](https://scholar.google.it/citations?user=skJbNQQAAAAJ&hl=it&oi=ao), 
 Blaz Repas, [Domenico Giustiniano](http://people.networks.imdea.org/~domenico_giustiniano/), [Vincent Lenders](http://www.lenders.ch/).

Precise Time-of-Arrival (TOA) estimations of aircraft and drone signals are 
important for a wide set of applications including aircraft/drone tracking, 
air traffic data verification, or self-localization. Our focus in this work 
is on TOA estimation methods that can run on low-cost software-defined radio 
(SDR) receivers, as widely deployed in Mode S / ADS-B crowdsourced sensor 
networks such as the [OpenSky Network](https://opensky-network.org/). 
We evaluate experimentally classical 
TOA estimation methods which are based on a cross-correlation with a 
reconstructed message template and find that these methods are not optimal 
for such signals. We propose two alternative methods that provide superior 
results for real-world Mode S / ADS-B signals captured with low-cost SDR 
receivers. The best method achieves a standard deviation error of 1.5 ns. 

# Features

* High Precision Time-of-Arrival (ToA) timestamps for Mode-S packets.
* Upsampling performs in the GPU (root is needed).
* A free-lock system queue (based on [moodycamel](https://github.com/cameron314/readerwriterqueue)) to parallelize message processing.
* Tested in extensively RaspberryPi-3 model B.
* Compilation process integrated with cmake.

# Building the project

* Clone the repository
```bash
$ git clone https://github.com/openskynetwork/dump1090-hptoa
```

* Compile the project
```bash
$ cd dump1090-hptoa/
$ mkdir build && cmake ../
$ make
```

# How to run

This new version adds a new parameter to enable the high precision timestamp method.

````
--enable-hptoa <method>  Enable High Precision TimeStamping: PeakPulse (default), CorrPulse or None
````

* PeakPulse: The default method with lower computation cost (no correlations are performed, only upsampling). The maximum message rate supported on RaspberryPi-3 model B on RPi 3  is estimated to be c.ca 2000 msg/sec.     We strongly recommend to use this method. 
  
* CorrPulse: Alternative method, slightly more precise than the previous one, but heavier on computation resources (CPU, memory). The maximum message rate supported on RaspberryPi-3 model B on RPi 3  is estimated around 700 msg/sec.     This option is provided to support future testing and research, we do not recommend to use it for production deployments. 

* None: No high-precision timestamp computed,  fallback to the legacy timestamp of the mutability version.

The detailed description of both methods can be found on the paper:  ['Nanosecond-precision Time-of-Arrival Estimation for Aircraft Signals with low-cost SDR Receivers'](http://eprints.networks.imdea.org/1768/)
 
## Run as root

Since the high precision timestamp estimation requires FFT/IFFT computation in the GPU, it is needed to run dump1090 process as root. This applies to both PeakPulse and CorrPulse method.


````
sudo dump1090 --enable-hptoa PeakPulse --interactive
````
