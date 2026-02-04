# Object rearrangement ros2 simulation

# Reference repositories
## Kuka descriptions:
```bash
kroshu/kuka_descriptions
```

## Kuka mobile base
[cogimon gazebo models](https://github.com/corlab/cogimon-gazebo-models.git)

# To pull the urdf models
```bash
git clone --filter=blob:none --no-checkout https://github.com/corlab/cogimon-gazebo-models.git
cd cogimon-gazebo-models
git sparse-checkout init --cone
git sparse-checkout set kuka-omnirob kuka-omnirob-lwrs
git checkout
mv kuka-omnirob ..
mv kuka-omnirob-lwrs ..
cd ..
rm -r cogimon-gazebo-models
```

# Download example world
```bash
curl -L -o moon_world.zip "https://fuel.gazebosim.org/1.0/anastasa/worlds/A%20base%20on%20the%20Moon.zip" && \
unzip moon_world.zip -d moon_world \
rm moon_world.zip
```