#!/usr/bin/env bash

# If any command fails, the script immediately exits
set -e

echo "Starting the Server Application..."
cargo run --bin server &

# Get the process ID of the last process (i.e., server)
SERVER_PID=$!

# Kill the server if it's already created and a subsequent command failed
trap "kill $SERVER_PID" EXIT

# give the server time to start up
# ideally, this should use retry logic instead
sleep 2 

echo "Starting the Client Application (runs the sim)..."
cargo run --bin client

echo "Generating plots using Julia..."
julia plotting/missile_engagment.jl

echo "Shutting down the server..."
kill $SERVER_PID