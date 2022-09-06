extern crate std;

use log::{debug, error, info, trace};
use structopt::StructOpt;

use tracing_subscriber::filter::{EnvFilter, LevelFilter};
use tracing_subscriber::FmtSubscriber;

use driver_pal::hal::{
    DeviceConfig, HalDelay, HalError, HalInputPin, HalInst, HalOutputPin, HalSpi,
};

use radio::*;
use radio_s2lp::*;

type Radio = S2lp<
    Io<HalSpi, HalOutputPin, HalOutputPin, HalDelay>,
    HalError,
    HalError,
    HalError,
>;

#[derive(Debug, PartialEq, Clone, StructOpt)]
pub enum Command {
    Info,

    #[structopt(flatten)]
    Other(helpers::Operation),
}

#[derive(Debug, StructOpt)]
pub struct Options {
    #[structopt(subcommand)]
    pub command: Command,

    #[structopt(flatten)]
    pub spi_config: DeviceConfig,

    #[structopt(long, default_value = "info")]
    /// Configure radio log level
    pub log_level: LevelFilter,
}

fn main() -> Result<(), anyhow::Error> {
    // Load options
    let opts = Options::from_args();

    // Initialise logging
    let filter = EnvFilter::from_default_env().add_directive(opts.log_level.into());
    let _ = FmtSubscriber::builder()
        .with_env_filter(filter)
        .without_time()
        .try_init();

    debug!("Connecting to platform SPI");

    let HalInst { base: _, spi, pins } = match HalInst::load(&opts.spi_config) {
        Ok(v) => v,
        Err(e) => {
            return Err(anyhow::anyhow!("Error connecting to platform HAL: {:?}", e));
        }
    };

    debug!("Setting up radio device");

    // Build IO object
    let io = Io {
        spi,
        cs: pins.cs,
        rst: pins.reset,
        // TODO: add appropriate pins to driver-pal or work out
        // how to better genericise / instantiate this
        //slp_tr: pins.led0,
        //irq: pins.ready,
        //clkm: Option::<()>::None,
        //dig2: Option::<()>::None,
        delay: HalDelay {},
    };

    // Build configuration
    let config = Config::default();

    // Instantiate radio
    let mut radio = match S2lp::new(io, config) {
        Ok(r) => r,
        Err(e) => {
            return Err(anyhow::anyhow!("Failed to initialise radio: {:?}", e));
        }
    };

    // Execute command
    if let Err(e) = handle_command(&mut radio, &opts.command) {
        return Err(anyhow::anyhow!("Command failed: {:?}", e));
    }

    Ok(())
}

fn handle_command(
    radio: &mut Radio,
    cmd: &Command,
) -> Result<(), Error<HalError, HalError, HalError>> {
    match cmd {
        Command::Info => {
            let i = radio.info()?;
            info!("Radio: {:02x?}", i);
        }
        Command::Other(op) => {
            if let Err(e) = helpers::do_operation(radio, op.clone()) {
                error!("Operation error: {:?}", e);
                return Err(Error::Unsupported);
            }
        }
    }

    Ok(())
}
