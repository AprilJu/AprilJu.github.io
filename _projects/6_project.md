---
layout: page
title: Humanoid robot locomotion with human reference
description: 
img: assets/img/mujoco.png
importance: 2
category: robotics
---

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/mujoco.png" title="Adam Lite simulation in MuJoCo" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Our team developed a novel imitation learning
framework for humanoid robotics, utilizing an adversarial motion prior to enhance
motion control. By designing and training control policies with Proximal Policy
Optimization (PPO) and leveraging NVIDIA's Isaac Gym for large-scale simulations, I integrated my understanding of robotics from
individual modules into a cohesive, efficient, and robust policy optimization
framework.


<!-- <video controls style="width: 100%; max-width: 600px; height: auto; display: block; margin: 1rem auto;">
  <source src="{{ '/assets/video/mimic_boxing.webm' | relative_url }}" type="video/webm">
  Your browser does not support the video tag.
</video> -->

<div style="display: flex; justify-content: center; gap: 1rem; flex-wrap: wrap;">
  <img src="{{ '/assets/img/mimic_boxing.gif' | relative_url }}" alt="Mimic Boxing" style="width: 250px; height: auto;">
  <img src="{{ '/assets/img/mimic_cycle.gif' | relative_url }}" alt="Mimic Cycle" style="width: 250px; height: auto;">
  <img src="{{ '/assets/img/mimic_swing.gif' | relative_url }}" alt="Mimic Swing" style="width: 250px; height: auto;">
</div>




