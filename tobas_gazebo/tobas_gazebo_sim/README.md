# tobas_gazebo_sim

## メモ

### world ファイルの例

- [gz-sim](https://github.com/gazebosim/gz-sim/tree/ign-gazebo3/examples/worlds)
- [Fuel Latest Worlds](https://app.gazebosim.org/fuel/worlds)

### パッケージからの URI 指定 (dsv.in にパスを正しく設定する必要あり)

```xml
<include>
  <uri>package://${package_name}/models/${model_name}</uri>
</include>
```
