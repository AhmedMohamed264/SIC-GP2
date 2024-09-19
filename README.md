# SIC-GP2 - Smart Parking System

## Project Overview

The Smart Parking System is an innovative solution designed to manage and optimize parking spaces using real-time data and advanced technologies. This system integrates various components to provide an intuitive interface for monitoring and controlling parking space utilization.

## Features

- **Gauge Visualization**: Provides a visual representation of parking space availability, updating in real-time based on user interactions.
- **Button Control**: Allows users to interact with individual parking spaces. Each button represents a parking space and can be activated or deactivated based on user input.
- **Dropdown Selection**: Enables users to specify the number of hours for parking, which influences the system's behavior and data updates.
- **Real-Time MQTT Communication**: Utilizes MQTT protocol for seamless communication between RFID readers and the Node-RED system, ensuring real-time updates and interactions.
- **Dynamic State Management**: The system ensures that buttons and gauges start in a default empty state, providing a clear and consistent user experience.

## Technologies Used

- **Node-RED**: For creating and managing the flow-based logic and handling MQTT messages.
- **MQTT**: Protocol used for efficient message exchange between devices and the Node-RED system.
- **Aedes Broker**: Lightweight MQTT broker that manages message brokering for the system.
- **JavaScript**: Employed in Node-RED for implementing custom logic and processing MQTT messages.

## Use Cases

- **Parking Space Monitoring**: Track and visualize available parking spaces in real-time.
- **Interactive Parking Management**: Manage parking spaces through a user-friendly interface with buttons and gauges.
- **Real-Time Notifications**: Receive instant updates about parking space status and availability through MQTT messages.
