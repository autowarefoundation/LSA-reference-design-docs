# ECUs

(To be completed)

The page lists the ECUs being tested for LSA, which can meet the computation capacity requirement of LSA but does not overprovision the resource requirements. 

Another page on [Autoware Document](https://autowarefoundation.github.io/autoware-documentation/main/reference-hw/ad-computers/) lists the ECUs being for other use scenarios. 

(Candidates of the ECUS to be used by the LSA)

## **nVidia In-Vehicle Computers**
![ad_comp-adlink.png](images/ad_comp-nvidia.jpg)
nVidia provides the development kit as a reference design. 


| Supported Products List | CPU                     | GPU                        | RAM, Interfaces                                 | Environmental | Autoware Tested (Y/N) |
| ----------------------- | ----------------------- | -------------------------- | ----------------------------------------------- | ------------- | --------------------- |
| nVidia Jetson Orin             | 12-core Arm® Cortex®-A78AE v8.2 64-bit CPU 3MB L2 + 6MB L3 | 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores | 64 GB RAM, Up to 6 cameras (16 via virtual channels), 4x USB2.0 1x 1GbE, 1x10GbE | None      | Yes                     |
| DRIVE AGX Orin Developer Kit   | 12 Cortex-A78A CPU | 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores | 16x GMSL cameras, 2x 10GbE, 10x 1GbE, 6x 100 MbE, 6 CAN  |  ASIL-D    | (TBA)                     |

Link to company website is [here.](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)


## **ADLINK In-Vehicle Computers**

![ad_comp-adlink.png](images/ad_comp-adlink.png)

ADLINK solutions which is used for autonomous driving and tested by one or more community members are listed below:

  <!-- cspell: ignore Altra BLUEBOX Grms Quadro Vecow vecow -->

| Supported Products List         | CPU                                    | GPU                      | RAM, Interfaces                                                                                    | Environmental                                                                                  | Autoware Tested (Y/N) |
| ------------------------------- | -------------------------------------- | ------------------------ | -------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- | --------------------- |
| ADM-AL30                        | Intel® 13/12th Gen Core™ I processor | Nvidia RTX 4000 SFF Ada  | 128GB RAM, Serial, USB, Automotive Ethernet (Base-T1), 10G Ethernet, CAN 2.0/ CAN-FD, M.2/SATA SSD | 9~36 VDC, E-Mark, 7637-2, IEC 60068-2-64: Operating: 5Grms, random, 5-500Hz, 3 axes (with SSD) | Y                     |
| AVA-3510                        | Intel® Xeon® E-2278GE                | Dual MXM RTX 5000        | 64GB RAM,CAN, USB, 10G Ethernet, DIO, Hot-Swap SSD, USIM                                           | 9~36 VDC, MIL-STD-810H,ISO 7637-2                                                              | Y                     |
| SOAFEE’s AVA Developer Platform | Ampere Altra ARMv8                     | optional                 | USB, Ethernet, DIO, M.2 NVMe SSDs                                                                  | 110/220 AC                                                                                     | Y                     |
| RQX-58G                         | 8-core Arm                             | Nvidia Jetson AGX Xavier | USB, Ethernet, M.2 NVME SSD, CAN, USIM, GMSL2 Camera support                                       | 9~36VDC, IEC 60068-2-64: Operating 3Grms, 5-500 Hz, 3 axes                                     | Y                     |
| RQX-59G                         | 8-core Arm                             | Nvidia Jetson AGX Orin   | USB, Ethernet, M.2 NVME SSD, CAN, USIM, GMSL2 Camera support                                       | 9~36VDC, IEC 60068-2-64: Operating 3Grms, 5-500 Hz, 3 axes                                     | -                     |

Link to company website is [here.](https://www.adlinktech.com/en/Connected-Autonomous-Vehicle-Solutions)

## **NXP In-Vehicle Computers**

![ad_comp-nxp.png](images/ad_comp-nxp.png)

NXP solutions which is used for autonomous driving and tested by one or more community members are listed below:

| Supported Products List | CPU                     | GPU                        | RAM, Interfaces                                 | Environmental | Autoware Tested (Y/N) |
| ----------------------- | ----------------------- | -------------------------- | ----------------------------------------------- | ------------- | --------------------- |
| BLUEBOX 3.0             | 16 x Arm® Cortex®-A72 | Dual RTX 8000 or RTX A6000 | 16 GB RAM CAN, FlexRay, USB, Ethernet, DIO, SSD | ASIL-D        | -                     |

Link to company website is [here.](https://www.nxp.com/design/designs/bluebox-3-0-automotive-high-performance-compute-ahpc-development-platform:BlueBox)

## **AdvanTech In-Vehicle Computers**


