# ros wiki

## try docker

docker ubuntu:16.04 使用して、ros 使用。xtermでターミナル複数開いて、gazeboと

status:
  - install ros :インストールは出来てそう 

  - executing...some issues : まだ実行時にエラーが
    - gazebo:  X DISPLAY property mismatch
      - [gazebo で X Error](https://github.com/naruai/wiki/blob/master/ros/errlog_gazebo.txt)
    - launch:  some error
      - [bash scripts/sim_with_judge.sh でもエラー](https://github.com/naruai/wiki/blob/master/ros/errlog_sim_with_judge.txt)

  - execute...log : なんか動いてる？
    - [log_your_burger_random.txt](https://github.com/naruai/wiki/blob/master/ros/log_your_burger_random.txt)

### 参考link

  - [GiHub OneNightROBOCON/burger_war](https://github.com/OneNightROBOCON/burger_war/blob/master/README.md)
    - [GiHub gogo5nta/burger_war](https://github.com/gogo5nta/burger_war/blob/master/README.md)
  - [Dockerコンテナの中でGUIアプリケーションを起動させる](https://unskilled.site/docker%E3%82%B3%E3%83%B3%E3%83%86%E3%83%8A%E3%81%AE%E4%B8%AD%E3%81%A7gui%E3%82%A2%E3%83%97%E3%83%AA%E3%82%B1%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E3%82%92%E8%B5%B7%E5%8B%95%E3%81%95%E3%81%9B%E3%82%8B/)

  - [ubuntu1804ros](https://symfoware.blog.fc2.com/blog-entry-2265.html)

#### 解決策探し中・・・

  - [Qiita DockerのUbuntuにデスクトップ環境を作成](https://qiita.com/FukuharaYohei/items/a6c1e1a2ba8d1285cfa5)
  - [Xサーバ設定ファイルxorg.confを設定する](http://www.ne.jp/asahi/it/life/it/linux/linux_setting/xorg_conf.html)
   
