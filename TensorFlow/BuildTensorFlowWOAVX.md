# Building TensorFlow without AVX instruction

 * Japanese language wiki [here. 日本語wikiはこっち](http://naruai.blog.jp/archives/988466.html)

## Celeron N4000 has no AVX instruction

Standard TensorFlow could not work on Celeron N4000!<br/>
~~~
$ python3 -c "import tensorflow as tf; print(tf.__version__)"
Illegal instruction (core dumped)
~~~
Standard TensorFlow need AVX supported CPU!<br/>
CPU of Lenovo Ideapad S130 was Celeron N4000 that no AVX support.<br/>
So I tried build TensorFlow that not need ATX instruction as below.<br/>
Then it worked as expected on Celeron N4000!<br/>

## Setup building environment with docker
~~~
$ cd ~/tfwork # work folder
$ mkdir ./dockerwork # blank folder
$ cd ./dockerwork
$ wget https://raw.githubusercontent.com/naruai/Dockerfile/master/BuildEnvTensorFlowPy3/Dockerfile
$ docker build -t ubuntu1804:py3dev .
~~~

## Extend swap
checking swap area
~~~
$ free -h
              total        used        free      shared  buff/cache   available
Mem:           3.7G        2.3G        131M        331M        1.3G        843M
Swap:          1.7G        456M        1.3G
~~~
need to extend
~~~
$ sudo mkdir /swapext #additional swapfile folder
$ cd /swapext
$ sudo fallocate -l 3G /swapext/swapfile2
$ sudo chmod 600 swapfile2 
$ sudo mkswap /swapext/swapfile2
$ sudo swapon /swapext/swapfile2
$ free -h
              total        used        free      shared  buff/cache   available
Mem:           3.7G        2.3G        138M        346M        1.3G        806M
Swap:          4.7G        461M        4.3G
~~~

## Download Bazel 0.25.2 (Can't be use ver 0.26.1, 0.27)
~~~
$ cd ~/tfwork # work folder
$ wget https://github.com/bazelbuild/bazel/releases/download/0.25.2/bazel-0.25.2-installer-linux-x86_64.sh
$ chmod +x bazel-0.25.2-installer-linux-x86_64.sh
~~~

## Build
~~~
$ docker run -v $PWD:/tfwork -it ubuntu1804:py3dev
~~~
Following is on docker.
~~~
# /tfwork/bazel-0.25.2-installer-linux-x86_64.sh 
# git clone https://github.com/tensorflow/tensorflow.git
~~~
If you will build another version of TensorFlow also, archive tensorflow folder and save it to host storage.<br/>
(In this example '/tfwork')<br/>
~~~
# cd tensorflow
# git checkout r1.14
Checking out files: 100% (4766/4766), done.
Branch 'r1.14' set up to track remote branch 'r1.14' from 'origin'.
Switched to a new branch 'r1.14'
# ls /usr/local/lib/python3.6/dist-packages/
Keras_Applications-1.0.6.dist-info   keras_applications    numpy                        six-1.12.0.dist-info
Keras_Preprocessing-1.0.5.dist-info  keras_preprocessing   numpy-1.16.4.dist-info       six.py
__pycache__                          libfuturize           past                         wheel
easy_install.py                      libpasteurize         pkg_resources                wheel-0.33.4.dist-info
future                               mock                  setuptools
future-0.17.1.dist-info              mock-3.0.5.dist-info  setuptools-41.0.1.dist-info
~~~
Keras_Applications found in /usr/local/lib/python3.6/dist-packages/ .<br/>
So select this path at configuring step below.<br/>
Other select is all default, so just ＜Enter＞ key.<br/>
~~~
# ./configure 
Extracting Bazel installation...
WARNING: --batch mode is deprecated. Please instead explicitly shut down your Bazel server using the command "bazel shutdown".
You have bazel 0.25.2 installed.
Please specify the location of python. [Default is /usr/bin/python]: 


Found possible Python library paths:
  /usr/lib/python3/dist-packages
  /usr/local/lib/python3.6/dist-packages
Please input the desired Python library path to use.  Default is [/usr/lib/python3/dist-packages]
~~~
Input <B>/usr/local/lib/python3.6/dist-packages</B> here
~~~
:
:
Preconfigured Bazel build configs. You can use any of the below by adding "--config=<>" to your build command. See .bazelrc for more details.
	--config=mkl         	# Build with MKL support.
	--config=monolithic  	# Config for mostly static monolithic build.
	--config=gdr         	# Build with GDR support.
	--config=verbs       	# Build with libverbs support.
	--config=ngraph      	# Build with Intel nGraph support.
	--config=numa        	# Build with NUMA support.
	--config=dynamic_kernels	# (Experimental) Build kernels into separate shared objects.
Preconfigured Bazel build configs to DISABLE default on features:
	--config=noaws       	# Disable AWS S3 filesystem support.
	--config=nogcp       	# Disable GCP support.
	--config=nohdfs      	# Disable HDFS support.
	--config=noignite    	# Disable Apache Ignite support.
	--config=nokafka     	# Disable Apache Kafka support.
	--config=nonccl      	# Disable NVIDIA NCCL support.
Configuration finished
~~~
Then build TensorFlow.<br/>
~~~
# bazel build --local_ram_resources=2048 --cxxopt="-D_GLIBCXX_USE_CXX11_ABI=0" --config=opt --config=noaws --config=nonccl //tensorflow/tools/pip_package:build_pip_package
~~~
<B>This step need much time...wait 1 day</B><br/>
~~~
# ./bazel-bin/tensorflow/tools/pip_package/build_pip_package /tmp/tensorflow_pkg
# cp /tmp/tensorflow_pkg/tensorflow-1.14.0-cp36-cp36m-linux_x86_64.whl /tfwork/
~~~

## Check Tensorflow with new whl file

Open another terminal on host.<br/>
~~~
$ cd ~/tfwork # work folder
$ pip install tensorflow-1.14.0-cp36-cp36m-linux_x86_64.whl
$ python3 -c "import tensorflow as tf; print(tf.__version__)"
1.14.0
~~~
'1.14.0' then success!
