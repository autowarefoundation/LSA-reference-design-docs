# Software Configuration

For Low Speed Autonomy, please refer to the official Autoware installation guide.
Both Docker-based and source-based installations are fully supported and work reliably.

## rmw_zenoh

With the latest ROS 2 Kilted release, Zenoh has become a Tier 1 middleware, marking the first non-DDS (Data Distribution Service) implementation to achieve this level of support.

Although rmw_zenoh is officially supported in Kilted, it has also been backported to both ROS 2 Jazzy and Humble. This means that Autoware, which uses ROS 2 Humble, can now run seamlessly with rmw_zenoh.

### Advantages of Zenoh over DDS

1. Out-of-the-box configuration: rmw_zenoh comes with a default configuration optimized for most use cases, reducing setup complexity.
2. Improved performance: Based on this benchmark study, Zenoh consistently outperforms other protocols, especially in wireless environments.
3. Cellular network support: Unlike DDS, which relies on multicast (often unsupported in 4G/5G), Zenoh works efficiently over unicast, making it suitable for mobile and cloud-connected scenarios.
4. Seamless cloud integration: DDS is typically restricted to local networks, whereas Zenoh can operate across the Internet, enabling easy data sharing with cloud systems.

We recommend using rmw_zenoh for Low Speed Autonomy applications to take advantage of its performance, simplicity, and modern network compatibility.

For a step-by-step tutorial on integrating rmw_zenoh with Autoware, please visit: [https://github.com/evshary/autoware_rmw_zenoh](https://github.com/evshary/autoware_rmw_zenoh)
