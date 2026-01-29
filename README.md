# Object rearrangement ros2 simulation

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