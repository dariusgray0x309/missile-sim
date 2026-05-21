using Dates
using DelimitedFiles
using Plots

scriptDir = @__DIR__
savePath = joinpath(scriptDir, "..", "sim_output")

timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")

# e.g. "plots/2025-04-18_21-15-32"
folder = joinpath(savePath, "plots", timestamp)
mkpath(folder)

dataPath = joinpath(savePath, "example_01.txt")
data = readdlm(dataPath, skipstart=1) # comma separated file

time    = Float64.(data[:, 1])
m_x     = Float64.(data[:, 2])  
m_y     = Float64.(data[:, 3]) 
t_x     = Float64.(data[:, 4])   
t_y     = Float64.(data[:, 5]) 
m_vx    = Float64.(data[:, 6]) 
m_vy    = Float64.(data[:, 7]) 
t_vx    = Float64.(data[:, 8])
t_vy    = Float64.(data[:, 9])
LOS     = Float64.(data[:, 10])  
LSR     = Float64.(data[:, 11])
range   = Float64.(data[:, 12]) 
cvel    = Float64.(data[:, 13])
m_accel = Float64.(data[:, 14])

# Range
p = plot(
    time, range,
    label="Range",
    xlabel="Time [s]", 
    ylabel="Range [m]", 
    title="Missile -> Target Range",
    lw=2
)

savefig(p, joinpath(folder, "range.png"))

# Closing Velocity
p = plot(
    time, cvel,
    label="Closing Velocity",
    xlabel="Time [s]", 
    ylabel="Range Rate [m/s]", 
    title="Closing Velocity",
    lw=2
)

savefig(p, joinpath(folder, "closingVelocity.png"))

# Line of Sight
p = plot(
    time, LOS,
    label="LOS",
    xlabel="Time [s]", 
    ylabel="LOS [rad]", 
    title="Line of Sight",
    lw=2
)

savefig(p, joinpath(folder, "LOS.png"))

# Line Of Sight Rate
p = plot(
    time, LSR,
    label="LSR",
    xlabel="Time [s]", 
    ylabel="LOS Rate [rad/s]", 
    title="Line of Sight Rate",
    lw=2
)

savefig(p, joinpath(folder, "LSR.png"))

# Missile Acceleration
p = plot(
    time, m_accel,
    label="Missile Acceleration",
    xlabel="Time [s]", 
    ylabel="Acceleration [G]", 
    title="Missile Acceleration",
    lw=2
)

savefig(p, joinpath(folder, "missileAcceleration.png"))

# Trajectory
plot(
    m_x, m_y,
    label="Missile",
    xlabel="Downrange [m]", 
    ylabel="Crossrange [m]", 
    title="Trajectory",
    legend=:bottomright,
    lw=2
)

plot!(
    t_x, t_y,
    label="Target",
    color = :red,
    linestyle = :dash,
    lw=2
)

savefig(joinpath(folder, "trajectory.png"))

println("Saved plots to ", folder, "\n")
