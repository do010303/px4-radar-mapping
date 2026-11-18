#!/usr/bin/env ruby
# frozen_string_literal: true

# `gz sim <args>` under another process title.
#
# Some simulation tooling on this machine stops Gazebo with `pkill -f 'gz sim'`,
# which would also hit this example's simulator even though it runs in its own
# transport partition. `gz` titles its process "gz sim ..."; this launcher runs
# the same command code but titles it "px4_mapping_gazebo ...".
#
#   ruby gz_launch.rb -s -r world.sdf      # server
#   ruby gz_launch.rb -g                   # GUI

require 'yaml'

conf = YAML.load_file(Dir.glob('/usr/share/gz/sim8.yaml').first || abort('gz-sim8 not installed'))
require conf['library_path']

Process.setproctitle("px4_mapping_gazebo #{ARGV.join(' ')}")
Cmd.new.execute(['sim'] + ARGV)
