# AIC Task Board Description

The **AI for Industry Challenge (AIC)** task board is a modular, reconfigurable platform designed to emulate real-world cable management challenges found in high-mix electronics manufacturing, specifically within server and data center infrastructure. This environment replicates the complex task of routing high-density fiber optics and seating transceivers into dense networking hardware.

This task board will serve here as the primary environment for evaluating dexterous manipulation, perception, and motion planning throughout the challenge phases.


## 1. Board Overview

The task board provides a standardized physical interface for the manipulation of **[SFP (Small Form-factor Pluggable)modules, LC (Lucent Connector) fiber optic and SC (Subscriber Connector) fiber optic](https://www.versitron.com/pages/sfp-sc-and-lc-connectors-transceivers-defined-and-analyzed-in-detail)** connectors. It is divided into four distinct zones to separate the assembly targets from the initial component pick locations.

![AIC Task Board](../../media/aic_task_board.png)

## 2. Zone Descriptions

The AIC task board is organized into four functional zones that simulate a complete electronics assembly workflow. Zones 1 and 2 serve as the assembly targets:
* [Zone 1](#zone-1-network-interface-cards-nic) houses the Network Interface Cards (NIC) with SFP ports, representing the server compute tray,
* [Zone 2](#zone-2-sc-optical-ports) mimics an optical patch panel with SC ports.
* [Zones 3](#zone-3--4-pick-locations) and 4 act as the Pick Locations, providing a high-mix supply area where SFP modules and fiber optic plugs are staged on adjustable mounts.

This modular layout requires the robot to transition from organized picking from zones 3 and 4 to precise, dexterous insertion in zones 1 and 2.

### Zone 1: Network Interface Cards (NIC)
This zone represents the networking switch or server compute tray where data links are established.

![AIC Task Board](../../media/aic_board_zone_1.png)

* **Components:** Supports up to five dual-port network cards (NIC).
* **Ports:** Each card features two SFP ports.
* **Flexibility:** Cards are designed to slide along mounting rails to allow for randomized positional and orientation offsets.
  * Card translation limits: [-0.0215, 0.0234] meters,
  * Card orientation limits: [-10, +10] degrees.

![AIC Task Board](../../media/aic_board_zone_1_legend.png)

### Zone 2: SC Optical Ports
This zone emulates the optical patch panel or backplane of a server rack.

![AIC Task Board](../../media/aic_board_zone_2.png)

* **Ports:** Supports up to five SC ports, distributed across two rails.
* **Flexibility:** Ports can slide along their rails to allow for randomized positional offsets.
  * SC port translation limits: [-0.06, 0.055] meters

![AIC Task Board](../../media/aic_board_zone_2_legend.png)

### Zone 3 & 4: Pick Locations
Zones 3 and 4 serve as organized supply areas for components (LC plugs, SC plugs, and SFP modules) before they are routed and inserted.

![AIC Task Board](../../media/aic_board_zone_3.png)
![AIC Task Board](../../media/aic_board_zone_4.png)

* **Mounts:** Holds fixtures for LC/SC plugs and SFP modules.
* **Customization:** Fixtures can be placed on any rail in any order, creating a high-mix environment.
  * Fixture translation limits: [-0.09425, 0.09425] meters
  * Fixture orientation limits: [-60, +60] degrees


![AIC Task Board](../../media/aic_board_zone_3_legend_1.png)
![AIC Task Board](../../media/aic_board_zone_3_legend_2.png)

![AIC Task Board](../../media/aic_board_zone_4_lengend_1.png)
![AIC Task Board](../../media/aic_board_zone_4_legend_2.png)

## 3. Bill of Material (BOM)

The task board is designed to be easy to 3D print with readily available components. To build a complete task board, you will need:

* **Off-the-shelf components:**
  * NIC Card (Quantity: 5) - [Amazon link](https://a.co/d/5lkWCj4)
  * SFP module (Quantity: 5) - [Amazon link](https://a.co/d/7RGkdZO)
  * LC to SC cable (Quantity: 5) - [Amazon link](https://a.co/d/edbwgg2)
  * SC-SC connectors (Quantity: 1 pack) - [Amazon link](https://a.co/d/4PgnstS)
* **Task board BOM:** *3D printed chassis, rails, and component mounts.* **TODO**

## 4. Configuration Structure

The state of the board for each trial is defined via a YAML configuration. This allows the system to randomize the position and orientation of components within the rail limits specified above, ensuring the robot must rely on perception rather than hard-coded coordinates. See the AIC engine [sample config](../aic_engine/config/sample_config.yaml)
