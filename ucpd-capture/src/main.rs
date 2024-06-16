use analyzer_cbor::{AnalyserData, Decoder};
use byteorder::{ByteOrder, LittleEndian};
use clap::Parser;
use std::{
    fmt::Display,
    fs::File,
    io::{Read, Write},
    path::PathBuf,
    time::Duration,
};
use uom::si::{electric_current::milliampere, electric_potential::millivolt, power::milliwatt};
use usb_pd::{
    header::Header,
    messages::{
        pdo::{PowerDataObject, Request, SourceCapabilities},
        Message,
    },
};

fn print_pdo(pdo: &PowerDataObject) {
    match pdo {
        PowerDataObject::FixedSupply(s) => println!(
            "  - Fixed: {:>5.2}V @ {:.2}A",
            s.voltage().get::<uom::si::electric_potential::millivolt>() as f64 / 1000.0,
            s.max_current().get::<milliampere>() as f64 / 1000.0,
        ),
        PowerDataObject::Battery(bat) => println!(
            "  - Variable: {:>5.2}V  - {:>5.2}V @ {:.2}W",
            bat.min_voltage().get::<uom::si::electric_potential::volt>() as f64 / 1000.0,
            bat.max_voltage().get::<millivolt>() as f64 / 1000.0,
            bat.max_power().get::<milliwatt>() as f64 / 1000.0,
        ),
        PowerDataObject::VariableSupply(var) => println!(
            "  - Variable: {:>5.2}V  - {:>5.2}V @ {:.2}A",
            var.min_voltage().get::<millivolt>() as f64 / 1000.0,
            var.max_voltage().get::<millivolt>() as f64 / 1000.0,
            var.max_current().get::<milliampere>() as f64 / 1000.0,
        ),
        PowerDataObject::AugmentedPowerDataObject(augmented) => match augmented {
            usb_pd::messages::pdo::AugmentedPowerDataObject::SPR(spr) => {
                println!(
                    "  - SPR PPS: {:>5.2}V - {:>5.2}V @ {:2.}A",
                    spr.min_voltage()
                        .get::<uom::si::electric_potential::millivolt>() as f64
                        / 1000.0,
                    spr.max_voltage()
                        .get::<uom::si::electric_potential::millivolt>() as f64
                        / 1000.0,
                    spr.max_current()
                        .get::<uom::si::electric_current::milliampere>() as f64
                        / 1000.0
                )
            }
            usb_pd::messages::pdo::AugmentedPowerDataObject::EPR(epr) => {
                println!(
                    "  - EPR PPS: {:>5.2}V - {:>5.2}V @ {}@",
                    epr.min_voltage().get::<millivolt>() as f64 / 1000.0,
                    epr.max_voltage().get::<millivolt>() as f64 / 1000.0,
                    epr.pd_power().get::<milliwatt>() as f64 / 1000.0,
                )
            }
            usb_pd::messages::pdo::AugmentedPowerDataObject::Unknown(u) => {
                println!("  - Unknown Augmented PDO: {u}")
            }
        },
        _ => println!("\t\t{pdo:?}"),
    }
}

fn print_source_capabilities(caps: &SourceCapabilities) {
    println!("SourceCapabilities:");
    println!("  Dual Role Power: {}", caps.dual_role_power());
    println!("  Usb suspend supported: {}", caps.usb_suspend_supported());
    println!("  Unconstrained power: {}", caps.unconstrained_power());
    println!("  Dual Role Data: {}", caps.dual_role_data());
    println!(
        "  Unchunked extended messages: {}",
        caps.unchunked_extended_messages_supported()
    );
    println!("  EPR mode capable: {}", caps.epr_mode_capable());
    println!("  PDOs:");
    for p in caps.pdos() {
        print_pdo(p);
    }
}

fn print_request(req: &Request, state: &Option<SourceCapabilities>) {
    let position = req.object_position().saturating_sub(1) as usize;
    if let Some(selected) = state.as_ref().and_then(|caps| caps.pdos().get(position)) {
        println!("  Selected:");
        print_pdo(selected)
    }
    print!("  Requested: ");
    match req {
        Request::FixedSupply(fixed) => println!(
            "{:.2}A (max {:.2}A)",
            fixed.operating_current().get::<milliampere>() as f64 / 1000.0,
            fixed.max_operating_current().get::<milliampere>() as f64 / 1000.0
        ),
        Request::VariableSupply(var) => println!(
            "{:.2}A (max {:.2}A)",
            var.operating_current().get::<milliampere>() as f64 / 1000.0,
            var.max_operating_current().get::<milliampere>() as f64 / 1000.0
        ),
        Request::Battery(bat) => println!(
            "{:.2}W (max {:.2}W)",
            bat.operating_power().get::<milliwatt>() as f64 / 1000.0,
            bat.max_operating_power().get::<milliwatt>() as f64 / 1000.0
        ),
        Request::PPS(pps) => println!(
            "{:.2}V @ {:.2}A",
            pps.output_voltage().get::<millivolt>() as f64 / 1000.0,
            pps.operating_current().get::<milliampere>() as f64 / 1000.0
        ),
        Request::AVS(avs) => println!(
            "{:.2}V @ {:.2}A",
            avs.output_voltage().get::<millivolt>() as f64 / 1000.0,
            avs.operating_current().get::<milliampere>() as f64 / 1000.0
        ),
        Request::Unknown(_) => todo!(),
    }
}

fn print_pd_message(message: &Message, state: &Option<SourceCapabilities>) {
    match message {
        //Message::Accept => todo!(),
        //Message::Reject => todo!(),
        //Message::Ready => todo!(),
        Message::SourceCapabilities(caps) => print_source_capabilities(caps),
        Message::Request(req) => print_request(req, state),
        //Message::VendorDefined(_) => todo!(),
        //Message::SoftReset => todo!(),
        _ => println!("Message: {:#?}", message),
    }
}

fn do_work<R: Read, W: Write>(mut input: R, mut output: Option<W>, show: DisplayInfo) {
    let mut data = [0u8; 1024];
    let mut d = Decoder::new(&mut data);
    let mut caps: Option<SourceCapabilities> = None;
    loop {
        let read_buffer = d.write_buffer();

        let r = input.read(read_buffer).unwrap();
        if r == 0 {
            return;
        }
        if let Some(ref mut output) = output {
            output
                .write_all(&read_buffer[0..r])
                .expect("Failed to write to output");
        }
        d.written(r);

        while let Some(data) = d.decode() {
            if !show.should_show(&data) {
                continue;
            }
            match data {
                AnalyserData::Version { version } => println!("Analyser version: {version}"),
                AnalyserData::PdSpy { data } => {
                    let header = Header(LittleEndian::read_u16(&data[..2]));
                    println!("======== PD Msg: {:?} ============", header.message_type());
                    println!("Header: {:#?}", header);
                    let message = Message::parse_with_state(header, &data[2..], &caps);
                    print_pd_message(&message, &caps);
                    if let Message::SourceCapabilities(c) = message {
                        caps = Some(c);
                    }
                }
                AnalyserData::Power {
                    label,
                    voltage,
                    current,
                } => {
                    print!("* Power: {label}: {voltage:.2}V");
                    if let Some(current) = current {
                        print!(" {current:.2}A");
                    }
                    println!();
                }
            }
        }
    }
}

#[derive(Copy, Clone, clap::ValueEnum, Debug, Default)]
enum DisplayInfo {
    #[default]
    PdOnly,
    PowerOnly,
    All,
    Nothing,
}

impl DisplayInfo {
    fn should_show(self, d: &AnalyserData<'_>) -> bool {
        matches!(
            (self, d),
            (Self::All, _)
                | (Self::PdOnly, AnalyserData::PdSpy { .. })
                | (Self::PowerOnly, AnalyserData::Power { .. })
        )
    }
}

impl Display for DisplayInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DisplayInfo::PdOnly => write!(f, "pd-only"),
            DisplayInfo::PowerOnly => write!(f, "power-only"),
            DisplayInfo::All => write!(f, "all"),
            DisplayInfo::Nothing => write!(f, "nothing"),
        }
    }
}

#[derive(Parser, Debug)]
struct Opts {
    #[clap(short, long)]
    output: Option<PathBuf>,
    #[clap(short, long)]
    from_file: bool,
    #[clap(short, long, default_value_t=DisplayInfo::PdOnly)]
    show: DisplayInfo,
    input: PathBuf,
}

fn main() {
    let opts = Opts::parse();
    let output = opts
        .output
        .map(|o| File::create(o).expect("Failed to open output file"));

    if opts.from_file {
        let input = File::open(opts.input).expect("Failed to open input file");
        do_work(input, output, opts.show);
    } else {
        let port = serialport::new(opts.input.to_str().unwrap(), 115200)
            .timeout(Duration::from_secs(u32::MAX as u64))
            .open()
            .expect("Failed to open serial port");
        do_work(port, output, opts.show);
    }
}
