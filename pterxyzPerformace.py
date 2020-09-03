
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time
# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Input_Output.Results import print_parasite_drag, \
    print_compress_drag, \
    print_engine_data, \
    print_mission_breakdown, \
    print_weight_breakdown
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
#from SUAVE.Input_Output.OpenVSP import vsp_write
#from SUAVE.Input_Output.OpenVSP import vspaero


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plot results    
    plot_mission(results)

    return


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    # vehicle data
    vehicle = vehicle_setup()
    configs = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission = mission_setup(configs_analyses, vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_UAV()
    weights.settings.empty_weight_method = \
        SUAVE.Methods.Weights.Correlations.Human_Powered.empty
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors  # what is called throughout the mission (at every time step))
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    # done!
    return analyses


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Pteryxyz_Electric'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.takeoff         = 450. * Units.kg
    vehicle.mass_properties.operating_empty = 450. * Units.kg
    vehicle.mass_properties.max_takeoff     = 450. * Units.kg

    # basic parameters
    vehicle.reference_area                    = 6.2
    vehicle.envelope.ultimate_load            = 2.0
    vehicle.envelope.limit_load               = 1.5
    max_q                                     = 56. #m/s
    vehicle.envelope.maximum_dynamic_pressure = 0.5*1.225*(max_q**2.) #Max q

    # ------------------------------------------------------------------
    #  Landing Gear
    # ------------------------------------------------------------------
    # used for noise calculations
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"

    landing_gear.main_tire_diameter = 0.423 * Units.m
    landing_gear.nose_tire_diameter = 0.3625 * Units.m
    landing_gear.main_strut_length = 0.4833 * Units.m
    landing_gear.nose_strut_length = 0.3625 * Units.m
    landing_gear.main_units = 2  # number of main landing gear units
    landing_gear.nose_units = 1  # number of nose landing gear
    landing_gear.main_wheels = 1  # number of wheels on the main landing gear
    landing_gear.nose_wheels = 1  # number of wheels on the nose landing gear
    vehicle.landing_gear = landing_gear

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'main_wing'

    wing.areas.reference = vehicle.reference_area
    wing.spans.projected = 7.04 * Units.meter
    wing.aspect_ratio = (wing.spans.projected ** 2) / wing.areas.reference
    wing.sweeps.quarter_chord = 0.0 * Units.deg
    wing.symmetric = True
    wing.thickness_to_chord = 0.12
    wing.taper = 1.0
    wing.vertical = False
    wing.high_lift = True
    wing.dynamic_pressure_ratio = 1.0
    wing.chords.mean_aerodynamic = wing.areas.reference / wing.spans.projected
    wing.chords.root = wing.areas.reference / wing.spans.projected
    wing.chords.tip = wing.areas.reference / wing.spans.projected
    wing.span_efficiency = 0.98
    wing.twists.root = 0.0 * Units.degrees
    wing.twists.tip = 0.0 * Units.degrees
    wing.highlift = False
    wing.vertical = False
    wing.number_ribs = 26.
    wing.number_end_ribs = 2.
    wing.transition_x_upper = 0.6
    wing.transition_x_lower = 1.0
    wing.origin = [1.6, 0.0, 0.0]  # meters
    wing.aerodynamic_center = [1.9, 0.0, 0.0]  # meters

    # add to vehicle
    vehicle.append_component(wing)
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.spans.projected = 2.45 * Units.meter
    wing.areas.reference = 1.20
    wing.aspect_ratio = (wing.spans.projected ** 2) / wing.areas.reference
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord = 0.12
    wing.taper = 1.0
    wing.span_efficiency = 0.95

    wing.areas.wetted = 2.0 * wing.areas.reference
    wing.areas.exposed = 0.8 * wing.areas.wetted
    wing.areas.affected = 0.6 * wing.areas.wetted
    wing.twists.root = 0.0 * Units.degrees
    wing.twists.tip = 0.0 * Units.degrees

    wing.vertical = False
    wing.symmetric = True
    wing.dynamic_pressure_ratio = 0.9
    wing.number_ribs = 5.0
    wing.chords.root = wing.areas.reference / wing.spans.projected
    wing.chords.tip = wing.areas.reference / wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference / wing.spans.projected
    wing.origin = [4.2, 0.0, 0.0]  # meters
    wing.aerodynamic_center = [0.5, 0.0, 0.0]  # meters

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'

    wing.spans.projected = 4.60 * Units.meter
    wing.areas.reference = 0.5
    wing.aspect_ratio = (wing.spans.projected ** 2) / wing.areas.reference
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord = 0.12
    wing.taper = 1.0
    wing.span_efficiency = 0.97
    wing.areas.reference = vehicle.reference_area * 0.1
    wing.spans.projected = np.sqrt(wing.aspect_ratio * wing.areas.reference)

    wing.chords.root = wing.areas.reference / wing.spans.projected
    wing.chords.tip = wing.areas.reference / wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference / wing.spans.projected
    wing.areas.wetted = 2.0 * wing.areas.reference
    wing.areas.exposed = 0.8 * wing.areas.wetted
    wing.areas.affected = 0.6 * wing.areas.wetted
    wing.twists.root = 0.0 * Units.degrees
    wing.twists.tip = 0.0 * Units.degrees
    wing.origin = [3.7, 0.0, 0.0]  # meters
    wing.aerodynamic_center = [0.5, 0.0, 0.0]  # meters
    wing.symmetric = True
    wing.vertical = True
    wing.t_tail = False
    wing.dynamic_pressure_ratio = 1.0
    wing.number_ribs = 5.

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()

    fuselage.tag = 'fuselage'

    # fuselage.aerodynamic_center= [2.986,0,1.077]

    fuselage.number_coach_seats = vehicle.passengers
    fuselage.seats_abreast = 2
    fuselage.seat_pitch = 0.995 * Units.meter
    fuselage.fineness.nose = 1.27
    fuselage.fineness.tail = 1  # 3.31
    fuselage.lengths.nose = 1.16 * Units.meter
    fuselage.lengths.tail = 4.637 * Units.meter
    fuselage.lengths.cabin = 2.653 * Units.meter
    fuselage.lengths.total = 8.45 * Units.meter
    fuselage.lengths.fore_space = 0.0 * Units.meter
    fuselage.lengths.aft_space = 0.0 * Units.meter
    fuselage.width = 1.22 * Units.meter
    fuselage.heights.maximum = 1.41 * Units.meter
    fuselage.effective_diameter = 2 * Units.meter
    fuselage.areas.side_projected = 7.46 * Units['meters**2']
    fuselage.areas.wetted = 25.0 * Units['meters**2']
    fuselage.areas.front_projected = 1.54 * Units['meters**2']
    fuselage.differential_pressure = 0.0 * Units.pascal  # Maximum differential pressure

    fuselage.heights.at_quarter_length = 1.077 * Units.meter
    fuselage.heights.at_three_quarters_length = 0.5 * Units.meter  # 0.621 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 1.41 * Units.meter


    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #  Propellers Powered By Batteries
    # ------------------------------------------------------------------    

    # build network
    net = Battery_Propeller()
    net.number_of_engines    = 1
    net.nacelle_diameter     = 0.58 * Units.meters
    net.engine_length        = 1.74 * Units.meters
    net.thrust_angle         = 0.0 * Units.degrees
    net.voltage              = 709.0 * Units.V
    net.areas                = Data()
    net.areas.wetted         = 1.1 * np.pi * net.nacelle_diameter * net.engine_length
    max_power                = 175 * Units.kW

    # Component 3 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95  # Gundlach for brushless motors
    net.esc = esc

    # Component 4 - Propeller

    # Design the Propeller
    # Component 5 the Propeller front
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_blades = 2.0
    prop.freestream_velocity = 56.0 * Units['m/s']  # freestream
    prop.angular_velocity = 10000. * Units['rpm']
    prop.tip_radius = 1.40 * Units.meters
    prop.hub_radius = 0.05 * Units.meters
    prop.design_Cl = 0.7
    prop.design_altitude = 5.0 * Units.km
    prop.design_thrust = None
    prop.design_power = 150000.0 * Units.watts
    prop = propeller_design(prop)

    net.propeller = prop

    # Component 4 the Motor front
    motor = SUAVE.Components.Energy.Converters.Motor()
    motor.resistance           = 0.01
    motor.no_load_current      = 125.  * Units.ampere
    motor.speed_constant       = 4.5* Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt
    motor.propeller_radius     = prop.tip_radius
    motor.propeller_Cp         = prop.power_coefficient
    motor.gear_ratio           = 1. # Gear ratio
    motor.gearbox_efficiency   = .98 # Gear box efficiency
    motor.expected_current     = 212.  # Expected current
    motor.mass_properties.mass = 50.0  * Units.kg
    net.motor                  = motor

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw = 50. * Units.watts
    payload.mass_properties.mass = 5.0 * Units.kg
    net.payload = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 50. * Units.watts
    net.avionics = avionics

    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()


    bat.cell_energy             = 9.36 #[Wh]
    bat.cell_capacity           = 2600 #[mAh]
    bat.max_const_discharge     = 35   #[A]
    bat.v_max                   = 4.2  #[V]
    bat.v_nom                   = 3.7  #[V]
    bat.v_min                   = 2.5  #[V]
    bat.cell_mass               = 44.3 #[g]
    bat.cell_current            = 2.5 #[A]
    bat.voltage                 = 709  #[V]
    bat.power                   = 150  #[kW]
    bat.boost_power             = 175  #[kW]
    bat.no_of_packs             = 2
    bat.current                 = bat.voltage * bat.power
    bat.boost_current           = bat.voltage * bat.boost_power
    bat.current_per_pack        = bat.current / bat.no_of_packs
    bat.parallel_cells_per_pack = 4
    bat.additional_cell         = 1
    bat.series_cells            = bat.voltage/bat.v_max
    bat.parallel_cells          = bat.parallel_cells_per_pack*bat.additional_cell
    bat.cells_per_pack          = bat.series_cells * bat.parallel_cells
    bat.cell_mass_ratio         = 1.2

    bat.mass_properties.mass    = bat.cell_mass_ratio * bat.cells_per_pack * bat.cell_mass /1000 * Units.kg
    bat.pack_energy             = bat.cells_per_pack * bat.cell_energy / 1000
    bat.total_energy            = bat.pack_energy * bat.no_of_packs

    bat.specific_energy = 450. * Units.Wh / Units.kg
    bat.resistance = 0.05
    initialize_from_mass(bat, bat.mass_properties.mass)
    net.battery = bat

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    # add the energy network to the vehicle
    vehicle.append_component(net)

    # print vehicle

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------
def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)

    return configs

def simple_sizing(configs):
    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff

    # wing areas
    for wing in base.wings:
        wing.areas.wetted = 2.0 * wing.areas.reference
        wing.areas.exposed = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()

    return


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses, vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'The Test Mission'

    mission.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet = SUAVE.Attributes.Planets.Earth()

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.propulsor.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.propulsor.residuals
    base_segment.state.unknowns.battery_voltage_under_load = vehicle.propulsors.propulsor.battery.max_voltage * ones_row(
        1)
    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.state.unknowns.propeller_power_coefficient = vehicle.propulsors.propulsor.propeller.power_coefficient * ones_row(
        1) / 15.
    base_segment.state.residuals.network = 0. * ones_row(2)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------

    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)

    # segment attributes
    segment.state.numerics.number_control_points = 100
    segment.start_time = time.strptime("Tue, Jun 21 11:30:00  2020", "%a, %b %d %H:%M:%S %Y", )
    segment.altitude = 1.0 * Units.km
    segment.mach           = 0.12
    segment.distance = 5.0 * Units.km
    segment.battery_energy = vehicle.propulsors.propulsor.battery.max_energy * 0.95  # Charge the battery to start
    segment.latitude = 37.4300  # this defaults to degrees (do not use Units.degrees)
    segment.longitude = -122.1700  # this defaults to degrees

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission


def missions_setup(base_mission):
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions


# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results):
    # ------------------------------------------------------------------
    #   Throttle
    # ------------------------------------------------------------------
    plt.figure("Throttle History")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        eta = results.segments[i].conditions.propulsion.throttle[:, 0]

        axes.plot(time, eta, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Throttle')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.ylim((0, 1))
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Altitude
    # ------------------------------------------------------------------
    plt.figure("Altitude")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        altitude = results.segments[i].conditions.freestream.altitude[:, 0] / Units.km
        axes.plot(time, altitude, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Altitude (km)')
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Forces")
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        Lift = -segment.conditions.frames.wind.lift_force_vector[:, 2]
        Drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

        axes = fig.add_subplot(3, 1, 1)
        axes.plot(time, Lift, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Lift (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 2)
        axes.plot(time, Drag, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 3)
        axes.plot(time, Thrust, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        plt.ylim((0, 50))
        axes.grid(True)

    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        CLift = segment.conditions.aerodynamics.lift_coefficient[:, 0]
        CDrag = segment.conditions.aerodynamics.drag_coefficient[:, 0]
        Drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

        axes = fig.add_subplot(3, 1, 1)
        axes.plot(time, CLift, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CL')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 2)
        axes.plot(time, CDrag, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CD')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 3)
        axes.plot(time, Drag, 'bo-')
        axes.plot(time, Thrust, 'ro-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:, 0]
        cdi = drag_breakdown.induced.total[:, 0]
        cdc = drag_breakdown.compressible.total[:, 0]
        cdm = drag_breakdown.miscellaneous.total[:, 0]
        cd = drag_breakdown.total[:, 0]

        axes.plot(time, cdp, 'ko-', label='CD_P')
        axes.plot(time, cdi, 'bo-', label='CD_I')
        axes.plot(time, cdc, 'go-', label='CD_C')
        axes.plot(time, cdm, 'yo-', label='CD_M')
        axes.plot(time, cd, 'ro-', label='CD')

        if i == 0:
            axes.legend(loc='upper center')

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Battery Energy
    # ------------------------------------------------------------------
    plt.figure("Battery Energy")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        energy = results.segments[i].conditions.propulsion.battery_energy[:, 0]
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Battery Energy (J)')
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Current Draw
    # ------------------------------------------------------------------
    plt.figure("Current Draw")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        energy = results.segments[i].conditions.propulsion.current[:, 0]
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Current Draw (Amps)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.ylim((0, 200))
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Motor RPM
    # ------------------------------------------------------------------
    plt.figure("Motor RPM")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        energy = results.segments[i].conditions.propulsion.rpm[:, 0]
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Motor RPM')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.ylim((0, 200))
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Battery Draw
    # ------------------------------------------------------------------
    plt.figure("Battery Charging")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        energy = results.segments[i].conditions.propulsion.battery_draw[:, 0]
        axes.plot(time, energy, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Battery Charging (Watts)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    axes.grid(True)

    # ------------------------------------------------------------------    
    #   Propulsive efficiency
    # ------------------------------------------------------------------
    plt.figure("Propeller Efficiency")
    axes = plt.gca()
    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        etap = results.segments[i].conditions.propulsion.etap[:, 0]
        axes.plot(time, etap, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Etap')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    axes.grid(True)
    plt.ylim((0, 1))

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.km
        mach = segment.conditions.freestream.mach_number[:, 0]
        aoa = segment.conditions.aerodynamics.angle_of_attack[:, 0] / Units.deg

        axes = fig.add_subplot(3, 1, 1)
        axes.plot(time, aoa, 'bo-')
        axes.set_ylabel('Angle of Attack (deg)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 2)
        axes.plot(time, altitude, 'bo-')
        axes.set_ylabel('Altitude (km)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        axes = fig.add_subplot(3, 1, 3)
        axes.plot(time, mach, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Mach Number (-)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True)

        # ------------------------------------------------------------------
    #  Solar Flux, Charging Power, Battery Energy
    # ------------------------------------------------------------------

    fig = plt.figure("Electric Outputs")
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        charge = results.segments[i].conditions.propulsion.battery_draw[:, 0]
        energy = results.segments[i].conditions.propulsion.battery_energy[:, 0] / Units.MJ



        axes = fig.add_subplot(3, 1, 1)
        axes.plot(time, energy, 'bo-')
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Battery Energy (MJ)')
        axes.grid(True)

    return


if __name__ == '__main__':
    main()
    plt.show()
