## Setting up Docker

```curl -fsSL https://get.docker.com -o get-docker.sh```

```sudo sh get-docker.sh```

```sudo usermod -aG docker $USER``` IMPORTANT: Log out and log back in for the group changes to apply or run ```newgrp docker```

```sudo apt-get update```

```sudo apt-get install -y docker-compose```

## Running the Docker Image

### docker-compose

```cd <top_project_folder>```

```docker-compose build --no-cache raspberry_pi```

```docker-compose up raspberry_pi```

### without docker-compose

```cd <top_project_folder>/raspberry_pi```

```docker build --no-cache -t raspberry_pi .```

```docker run -it --rm --privileged --network host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=/cyclonedds.xml -v "/cyclonedds.xml" raspberry_pi```

### Explanations:

```curl -fsSL https://get.docker.com -o get-docker.sh```

#### Explanation:

curl: This is the client URL tool, a command used to transfer data from or to a server.

-f (--fail): This flag tells curl to fail silently on server errors. If the server returns an HTTP error code (like 404 Not Found), curl won't output the server's error page and will simply exit with an error code. This is useful for scripting.

-s (--silent): This puts curl in silent mode, preventing it from showing progress meters or other non-essential messages.

-S (--show-error): This is used with -s. If the command fails (because of the -f flag), this flag makes curl show the error message, even in silent mode. So, it's silent on success but noisy on failure.

-L (--location): This flag tells curl to follow redirects. If the requested URL has been moved, the server will send back a new location, and -L ensures curl goes to that new URL to get the content.

https://get.docker.com: This is the URL where the Docker installation script is located.

-o get-docker.sh: This is the output flag. It tells curl to write the downloaded content to a file named get-docker.sh in your current directory, instead of printing it to the terminal.

### Run the script

```sudo sh get-docker.sh```

#### Explanation:

run the shell script abbreviated 'sh' to install docker

### Add user to User Group

```sudo usermod -aG docker $USER```
IMPORTANT: Log out and log back in for the group changes to apply!

#### Explanation:

usermod: The command to modify a user account's properties.

-a (append): This is crucial. It adds the user to the specified group without removing them from their current groups.

-G (Groups): Specifies the name of the group to add the user to.

docker: The name of the group. The Docker installation process creates this group specifically to manage Docker permissions.

'$'USER: This is an environment variable that automatically represents the username of the currently logged-in user. It's a convenient shorthand so you don't have to type your own username.

### Update the package manager

```sudo apt-get update```

#### Explanation:

apt-get: The command-line tool used to handle software packages on Debian-based Linux systems (like Ubuntu).

update: The specific action that tells apt-get to synchronize the local package index with the central software sources (repositories).

### Install docker-compose

```sudo apt-get install -y docker-compose```

#### Explanation:

Docker Compose is a tool that helps you define and run multi-container Docker applications. While Docker manages individual containers, Docker Compose manages an entire application stack (e.g., a web server, a database, and a caching service) at once.

It uses a simple YAML file (usually named docker-compose.yml) to configure all the application's services. With a single command, like docker-compose up, you can start, stop, and manage your entire application stack.
