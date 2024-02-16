this is a readme for our project


### Getting started

1. Connect to robot 
  ```
  ssh local@10.197.216.74
  ```

2. Login
  ```password: grenen```

3. Build
  ```
  cd svn/robobot/raubase/build && make
  ```

4. Run
  ```
  cd svn/robobot/raubase/build && ./raubase
  ```


### Tips and tricks
- Moving files from ```ssh``` to ```local```. For instance: 
  ```
  scp -r local@10.197.216.74:/home/local/svn<PATH_TO_FILE> <LOCAL_PATH>
  ```
  - For instance:
  ```
  scp -r local@10.197.216.74:/home/local/svn/robobot/raubase/build/img/ /Users/sebastianbitsch/Desktop/
  ```






