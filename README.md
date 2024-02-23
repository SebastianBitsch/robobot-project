### Getting started

1. Connect to robot 
  ```bash
  ssh local@10.197.216.74
  ```

2. Login
  ```password: grenen```

3. Build
  ```bash
  cd svn/robobot/raubase/build && make
  ```

4. Run
  ```bash
  cd svn/robobot/raubase/build && ./raubase
  ```


### Tips and tricks
- Moving files from ```ssh``` to ```local```. For instance: 
  ```bash
  scp -r local@10.197.216.74:/home/local/svn<PATH_TO_FILE> <LOCAL_PATH>
  ```
  - For instance:
  ```bash
  scp -r local@10.197.216.74:/home/local/svn/robobot/raubase/build/img/ /Users/sebastianbitsch/Desktop/
  ```


# Camera
Access the stream
```bash
cd svn/robobot/raubase/build/
motion
```
See the output stream at <http://10.197.216.74:8081>




