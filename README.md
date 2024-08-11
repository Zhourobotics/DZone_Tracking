# Multi-Robot Target Tracking with Sensing and Communication Danger Zones

## About 

__Authors__: [Jiazhen Liu*](https://scholar.google.com/citations?user=x4OzGCwAAAAJ&hl=en), [Peihan Li*](https://scholar.google.com/citations?user=Qg7-Gr0AAAAJ&hl=en), 
[Yuwei Wu](https://github.com/yuwei-wu), Gaurav S. Sukhatmem, and Vijay Kumar, and [Lifeng Zhou](https://zhourobotics.github.io/)

__Video Links__:  [Youtube](https://youtu.be/uSYPI817Y6c)

__Related Paper__: Jiazhen, Liu* Peihan Li*, Yuwei Wu, Gaurav S. Sukhatme, Vijay Kumar, and Lifeng Zhou. "Multi-Robot Target Tracking with Sensing and Communication Danger Zones." 2024 International Symposium on Distributed Autonomous Robotic Systems
[Preprint](https://arxiv.org/abs/2404.07880.pdf)


```
@misc{liu2024multirobottargettrackingsensing,
      title={Multi-Robot Target Tracking with Sensing and Communication Danger Zones}, 
      author={Jiazhen Liu and Peihan Li and Yuwei Wu and Gaurav S. Sukhatme and Vijay Kumar and Lifeng Zhou},
      year={2024},
      eprint={2404.07880},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2404.07880}, 
}
```

## Prerequisites

- [ROS](https://wiki.ros.org/ROS/Installation): Our framework has been tested in ROS Noetic.

- [Forces Pro](https://www.embotech.com/products/forcespro/overview/) and Matlab: You can request an academic license from [here](https://www.embotech.com/products/forcespro/licensing/).

## Usage

- In the simulation, run 'main.m'.
- Parameters are in config.json. You can change robots' and targets' numbers, start positions, etc.
- Type I represents sensing danger zones and Type II represents communication danger zones. You can set the position and covariance. 

## Maintaince

For any technical issues, please contact Jiazhen Liu (jliu3103@gatech.edu).
