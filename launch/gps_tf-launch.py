<launch>
  <!-- Launch the tf_broadcaster_node -->
  <node pkg="test2" exec="tf_broadcaster_node" name="tf_broadcaster_node" output="screen">
  </node>

  <!-- Launch the ins_to_ecef_enu_node -->
  <node pkg="test2" exec="ins_to_ecef_enu_node" name="ins_to_ecef_enu_node" output="screen">
  </node>
</launch>

