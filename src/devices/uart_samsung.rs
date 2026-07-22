use core::ptr;

use crate::{
    scheme::debug::{debug_input, debug_notify},
    sync::CleanLockToken,
};

// Register offsets shared by all Samsung UART generations. Control
// registers are always 32 bits wide; only UTXH/URXH access width varies
// per variant (see `Variant::wide_data`).
mod reg {
    pub const ULCON: usize = 0x00;
    pub const UCON: usize = 0x04;
    pub const UFCON: usize = 0x08;
    pub const UMCON: usize = 0x0c;
    pub const UTRSTAT: usize = 0x10;
    pub const UERSTAT: usize = 0x14;
    pub const UFSTAT: usize = 0x18;
    pub const UTXH: usize = 0x20;
    pub const URXH: usize = 0x24;
    pub const UBRDIV: usize = 0x28;
    /// UDIVSLOT on S3C6400/S5PV210, UFRACVAL on Exynos and newer.
    pub const UFRACVAL: usize = 0x2c;
    /// S3C64xx-style interrupt pending, write 1 to clear.
    pub const UINTP: usize = 0x30;
    /// S3C64xx-style interrupt source pending, write 1 to clear.
    pub const UINTSP: usize = 0x34;
    /// S3C64xx-style interrupt mask, 1 masks the source.
    pub const UINTM: usize = 0x38;
}

// Registers are read-modify-written as raw u32 values since several
// fields (clock select, Apple interrupt enables) are variant-specific
// and must be preserved rather than declared globally.
mod ulcon {
    /// 8 data bits, 1 stop bit, no parity.
    pub const CS8: u32 = 0x3;
}

mod ucon {
    pub const RXMODE_MASK: u32 = 0x3;
    pub const RXMODE_CPU: u32 = 0x1;
    pub const TXMODE_MASK: u32 = 0x3 << 2;
    pub const TXMODE_CPU: u32 = 0x1 << 2;
    /// A port with neither mode selected is unconfigured and cannot
    /// transmit or receive.
    pub const MODE_MASK: u32 = 0xf;
    pub const RXERR_IRQEN: u32 = 1 << 6;
    /// Called RXFIFO_TOI on S3C2410 and TIMEOUT_EN on S3C64xx.
    pub const RX_TIMEOUT_EN: u32 = 1 << 7;
    pub const RXINT_LEVEL: u32 = 1 << 8;
    pub const TXINT_LEVEL: u32 = 1 << 9;
    pub const DMASUS_EN: u32 = 1 << 10;
    pub const EMPTYINT_EN: u32 = 1 << 11;
    pub const RX_TIMEOUT_MASK: u32 = 0xf << 12;
    /// Longest receive timeout window, as programmed by Linux.
    pub const RX_TIMEOUT_MAX: u32 = 0xf << 12;
    pub const CLKSEL_S3C6400_MASK: u32 = 0x3 << 10;
    pub const CLKSEL_S5PV210_MASK: u32 = 0x1 << 10;
    // Apple S5L repurposes bits 9-13 as interrupt enables.
    pub const APPLE_RXTO_EN: u32 = 1 << 9;
    pub const APPLE_RXTO_LEGACY_EN: u32 = 1 << 11;
    pub const APPLE_RXTHRESH_EN: u32 = 1 << 12;
}

mod ufcon {
    pub const FIFO_EN: u32 = 1 << 0;
    pub const RX_RESET: u32 = 1 << 1;
    /// Self-clearing once the FIFO reset completes.
    pub const RESET_BOTH: u32 = 0x3 << 1;
    pub const S3C2410_RXTRIG8: u32 = 0x1 << 4;
    pub const S5PV210_RXTRIG4: u32 = 0x1 << 4;
    pub const S5PV210_RXTRIG8: u32 = 0x2 << 4;
    pub const S5PV210_TXTRIG4: u32 = 0x1 << 8;
}

mod umcon {
    pub const AUTO_FLOW: u32 = 1 << 4;
}

mod utrstat {
    pub const RX_READY: u32 = 1 << 0;
    /// Transmit buffer and shifter both empty.
    pub const TX_EMPTY: u32 = 1 << 2;
    pub const APPLE_RXTO_LEGACY: u32 = 1 << 3;
    pub const APPLE_RXTHRESH: u32 = 1 << 4;
    pub const APPLE_RXTO: u32 = 1 << 9;
    /// All Apple S5L interrupt flags (bits 9:3), write 1 to clear.
    pub const APPLE_ALL_FLAGS: u32 = 0x7f << 3;
    pub const APPLE_RX_FLAGS: u32 = APPLE_RXTHRESH | APPLE_RXTO | APPLE_RXTO_LEGACY;
}

mod uerstat {
    pub const OVERRUN: u32 = 1 << 0;
    pub const PARITY: u32 = 1 << 1;
    pub const FRAME: u32 = 1 << 2;
    pub const BREAK: u32 = 1 << 3;
    pub const ANY: u32 = OVERRUN | PARITY | FRAME | BREAK;
}

mod uint {
    pub const RXD: u32 = 1 << 0;
    pub const ALL: u32 = 0xf;
}

const UBRDIV_MAX: u32 = 0xffff;
const TX_IDLE_POLL_LIMIT: usize = 1_000_000;
const FIFO_RESET_POLL_LIMIT: usize = 10_000;
const RX_DRAIN_LIMIT: usize = 256;

/// UDIVSLOT patterns indexed by the divisor fraction in sixteenths,
/// as recommended by the S3C6400/S5PV210 manuals.
const UDIVSLOT_TABLE: [u16; 16] = [
    0x0000, 0x0080, 0x0808, 0x0888, 0x2222, 0x4924, 0x4A52, 0x54AA, 0x5555, 0xD555, 0xD5D5, 0xDDD5,
    0xDDDD, 0xDFDD, 0xDFDF, 0xFFDF,
];

/// UFSTAT field layout, which changed as FIFO depths grew.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FifoLayout {
    /// 16-byte FIFOs: count in bits 3:0/7:4, full flags at bits 8/9.
    S3c2410,
    /// 64-byte FIFOs: count in bits 5:0/13:8, full flags at bits 6/14.
    S3c2440,
    /// Up to 256-byte FIFOs: count in bits 7:0/23:16, full flags at
    /// bits 8/24.
    S5pv210,
}

impl FifoLayout {
    /// UFSTAT bits that are non-zero while the receive FIFO holds data.
    /// The full flag is included since the count field wraps to zero
    /// when the FIFO is completely full.
    const fn rx_pending_mask(self) -> u32 {
        match self {
            FifoLayout::S3c2410 => 0xf | 1 << 8,
            FifoLayout::S3c2440 => 0x3f | 1 << 6,
            FifoLayout::S5pv210 => 0xff | 1 << 8,
        }
    }

    const fn tx_full_mask(self) -> u32 {
        match self {
            FifoLayout::S3c2410 => 1 << 9,
            FifoLayout::S3c2440 => 1 << 14,
            FifoLayout::S5pv210 => 1 << 24,
        }
    }
}

/// How receive interrupts are enabled and acknowledged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IrqStyle {
    /// Dedicated UINTP/UINTM registers (S3C6400 through Exynos).
    S3c6400,
    /// Enable bits live in UCON, status is acknowledged through
    /// UTRSTAT (Apple S5L).
    AppleS5l,
}

/// How the baud-rate divisor is programmed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DivisorStyle {
    /// UBRDIV only; the divisor is rounded to the nearest integer.
    Integer,
    /// UBRDIV plus a UDIVSLOT pattern looked up from a table.
    Table,
    /// UBRDIV plus the fraction in sixteenths written to UFRACVAL.
    Fractional,
}

/// Differences between Samsung UART generations, selected from the
/// devicetree compatible list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Variant {
    fifo: FifoLayout,
    irq: IrqStyle,
    divisor: DivisorStyle,
    /// UTXH/URXH require 32-bit accesses instead of byte accesses.
    wide_data: bool,
    ucon_default: u32,
    /// UCON bits preserved across full initialization (clock select).
    ucon_preserve: u32,
    ufcon_default: u32,
}

const UCON_DEFAULT_S3C6400: u32 = ucon::TXINT_LEVEL
    | ucon::RXINT_LEVEL
    | ucon::TXMODE_CPU
    | ucon::RXMODE_CPU
    | ucon::RX_TIMEOUT_EN;
const UCON_DEFAULT_S5PV210: u32 = UCON_DEFAULT_S3C6400 | ucon::RXERR_IRQEN;
const UCON_DEFAULT_APPLE: u32 = ucon::TXMODE_CPU | ucon::RXMODE_CPU | ucon::RX_TIMEOUT_EN;

const UFCON_DEFAULT_S3C2410: u32 = ufcon::FIFO_EN | ufcon::S3C2410_RXTRIG8;
const UFCON_DEFAULT_S5PV210: u32 = ufcon::FIFO_EN | ufcon::S5PV210_TXTRIG4 | ufcon::S5PV210_RXTRIG4;

impl Variant {
    const S3C6400: Variant = Variant {
        fifo: FifoLayout::S3c2440,
        irq: IrqStyle::S3c6400,
        divisor: DivisorStyle::Table,
        wide_data: false,
        ucon_default: UCON_DEFAULT_S3C6400,
        ucon_preserve: ucon::CLKSEL_S3C6400_MASK,
        ufcon_default: UFCON_DEFAULT_S3C2410,
    };

    const S5PV210: Variant = Variant {
        fifo: FifoLayout::S5pv210,
        irq: IrqStyle::S3c6400,
        divisor: DivisorStyle::Table,
        wide_data: false,
        ucon_default: UCON_DEFAULT_S5PV210,
        ucon_preserve: ucon::CLKSEL_S5PV210_MASK,
        ufcon_default: UFCON_DEFAULT_S5PV210,
    };

    const EXYNOS: Variant = Variant {
        fifo: FifoLayout::S5pv210,
        irq: IrqStyle::S3c6400,
        divisor: DivisorStyle::Fractional,
        wide_data: false,
        ucon_default: UCON_DEFAULT_S5PV210,
        ucon_preserve: 0,
        ufcon_default: UFCON_DEFAULT_S5PV210,
    };

    const GS101: Variant = Variant {
        wide_data: true,
        ..Variant::EXYNOS
    };

    const APPLE_S5L: Variant = Variant {
        fifo: FifoLayout::S3c2410,
        irq: IrqStyle::AppleS5l,
        divisor: DivisorStyle::Integer,
        wide_data: true,
        ucon_default: UCON_DEFAULT_APPLE,
        ucon_preserve: 0,
        ufcon_default: UFCON_DEFAULT_S3C2410,
    };

    pub fn from_compatible(compatible: &str) -> Option<Variant> {
        // Compatible lists order SoC-specific values before fallbacks,
        // so the first recognized value wins.
        for value in compatible.split('\0') {
            let variant = match value {
                "samsung,s3c6400-uart" => Variant::S3C6400,
                "samsung,s5pv210-uart" => Variant::S5PV210,
                "samsung,exynos4210-uart"
                | "samsung,exynos5433-uart"
                | "samsung,exynos850-uart"
                | "samsung,exynos8895-uart"
                | "axis,artpec8-uart" => Variant::EXYNOS,
                "google,gs101-uart" => Variant::GS101,
                "apple,s5l-uart" => Variant::APPLE_S5L,
                _ => continue,
            };
            return Some(variant);
        }

        None
    }

    /// Applies the devicetree reg-io-width property, which overrides
    /// the variant's default data-register access width.
    pub fn with_reg_io_width(mut self, width: Option<usize>) -> Variant {
        match width {
            Some(1) => self.wide_data = false,
            Some(4) => self.wide_data = true,
            // Unknown widths keep the variant default; a possibly
            // wrong console beats no console.
            _ => {}
        }
        self
    }
}

pub fn is_compatible(compatible: &str) -> bool {
    Variant::from_compatible(compatible).is_some()
}

fn calculate_baud_registers(
    clock_rate: u32,
    baud_rate: u32,
    style: DivisorStyle,
) -> Option<(u32, Option<u32>)> {
    if baud_rate == 0 {
        return None;
    }

    match style {
        DivisorStyle::Integer => {
            let rounded = clock_rate
                .checked_add(baud_rate.checked_mul(8)?)?
                .checked_div(baud_rate.checked_mul(16)?)?;
            let ubrdiv = rounded.checked_sub(1)?;
            (ubrdiv <= UBRDIV_MAX).then_some((ubrdiv, None))
        }
        DivisorStyle::Table | DivisorStyle::Fractional => {
            // The divisor is kept in sixteenths of the bit clock: the
            // whole part goes to UBRDIV and the fraction is spread
            // over a 16-bit-time window by UDIVSLOT/UFRACVAL.
            let sixteenths = clock_rate.checked_div(baud_rate)?;
            let ubrdiv = (sixteenths / 16).checked_sub(1)?;
            let fraction = sixteenths & 15;
            let fracval = match style {
                DivisorStyle::Fractional => fraction,
                _ => u32::from(*UDIVSLOT_TABLE.get(fraction as usize)?),
            };
            (ubrdiv <= UBRDIV_MAX).then_some((ubrdiv, Some(fracval)))
        }
    }
}

pub struct SerialPort {
    base: usize,
    variant: Variant,
    baud_rate: u32,
    /// Baud clock input rate; unknown when the devicetree routes the
    /// clock through a programmable controller. In that case the
    /// firmware-provided baud rate and line format are preserved.
    clock_rate: Option<u32>,
    skip_init: bool,
}

impl SerialPort {
    /// # Safety
    ///
    /// `base` must be an aligned, mapped MMIO range of at least 0x3c
    /// bytes that exclusively represents a Samsung UART for the
    /// lifetime of the port.
    pub const unsafe fn new(
        base: usize,
        baud_rate: u32,
        clock_rate: Option<u32>,
        variant: Variant,
        skip_init: bool,
    ) -> SerialPort {
        SerialPort {
            base,
            variant,
            baud_rate,
            clock_rate,
            skip_init,
        }
    }

    fn read_reg(&self, register: usize) -> u32 {
        unsafe { ptr::read_volatile((self.base + register) as *const u32) }
    }

    fn write_reg(&self, register: usize, data: u32) {
        unsafe {
            ptr::write_volatile((self.base + register) as *mut u32, data);
        }
    }

    fn read_data(&self) -> u8 {
        unsafe {
            if self.variant.wide_data {
                ptr::read_volatile((self.base + reg::URXH) as *const u32) as u8
            } else {
                ptr::read_volatile((self.base + reg::URXH) as *const u8)
            }
        }
    }

    fn write_data(&self, data: u8) {
        unsafe {
            if self.variant.wide_data {
                ptr::write_volatile((self.base + reg::UTXH) as *mut u32, data as u32);
            } else {
                ptr::write_volatile((self.base + reg::UTXH) as *mut u8, data);
            }
        }
    }

    /// Makes the transmitter usable while preserving the serial
    /// configuration left by firmware. This can be called before the
    /// FIFO, baud-rate, and interrupt configuration has been validated.
    pub fn init_early(&mut self) {
        let ucon = self.read_reg(reg::UCON);
        if ucon & ucon::TXMODE_MASK == 0 {
            self.write_reg(reg::UCON, ucon | ucon::TXMODE_CPU);
        }
    }

    /// Resets FIFOs, masks interrupts, and installs the interrupt-mode
    /// defaults. The baud rate and 8N1 format are only programmed when
    /// the baud clock rate is known; otherwise the firmware line
    /// configuration is kept.
    pub fn init_full(&mut self) -> Result<(), ()> {
        if self.skip_init {
            return Ok(());
        }

        // Let the transmitter drain before the FIFOs are reset and the
        // line configuration changes.
        let mut idle = false;
        for _ in 0..TX_IDLE_POLL_LIMIT {
            if self.read_reg(reg::UTRSTAT) & utrstat::TX_EMPTY != 0 {
                idle = true;
                break;
            }
            core::hint::spin_loop();
        }
        if !idle {
            return Err(());
        }

        let mut ucon_value =
            (self.read_reg(reg::UCON) & self.variant.ucon_preserve) | self.variant.ucon_default;
        if self.variant.irq == IrqStyle::S3c6400 {
            ucon_value |= ucon::RX_TIMEOUT_MAX;
        }
        self.write_reg(reg::UCON, ucon_value);

        // Keep interrupts masked and acknowledge stale state; the
        // Apple enables were already cleared by the UCON write above.
        match self.variant.irq {
            IrqStyle::S3c6400 => {
                self.write_reg(reg::UINTM, uint::ALL);
                self.write_reg(reg::UINTP, uint::ALL);
                self.write_reg(reg::UINTSP, uint::ALL);
            }
            IrqStyle::AppleS5l => {
                self.write_reg(reg::UTRSTAT, utrstat::APPLE_ALL_FLAGS);
            }
        }

        // Reset both FIFOs; the reset bits clear themselves.
        self.write_reg(reg::UFCON, self.variant.ufcon_default | ufcon::RESET_BOTH);
        for _ in 0..FIFO_RESET_POLL_LIMIT {
            if self.read_reg(reg::UFCON) & ufcon::RESET_BOTH == 0 {
                break;
            }
            core::hint::spin_loop();
        }
        self.write_reg(reg::UFCON, self.variant.ufcon_default);

        // No auto flow control; an undriven CTS input would stall the
        // transmitter forever.
        let umcon_value = self.read_reg(reg::UMCON) & !umcon::AUTO_FLOW;
        self.write_reg(reg::UMCON, umcon_value);

        if let Some(clock_rate) = self.clock_rate {
            let (ubrdiv, fracval) =
                calculate_baud_registers(clock_rate, self.baud_rate, self.variant.divisor)
                    .ok_or(())?;
            self.write_reg(reg::ULCON, ulcon::CS8);
            self.write_reg(reg::UBRDIV, ubrdiv);
            if let Some(fracval) = fracval {
                self.write_reg(reg::UFRACVAL, fracval);
            }
        }

        Ok(())
    }

    /// Puts RX in interrupt mode and unmasks the receive interrupts.
    /// skip-init preserves the baud rate, format, and FIFO
    /// configuration supplied by firmware; interrupt setup still takes
    /// ownership of RX.
    pub fn enable_irq(&mut self) {
        let ufcon_value = self.read_reg(reg::UFCON);
        self.write_reg(
            reg::UFCON,
            ufcon_value | ufcon::RX_RESET | ufcon::S5PV210_RXTRIG8,
        );

        let mut ucon_value = self.read_reg(reg::UCON);
        ucon_value &= !ucon::RXMODE_MASK;
        ucon_value |= ucon::RXMODE_CPU;

        match self.variant.irq {
            IrqStyle::S3c6400 => {
                ucon_value &= !(ucon::RX_TIMEOUT_MASK | ucon::EMPTYINT_EN | ucon::DMASUS_EN);
                ucon_value |= ucon::RX_TIMEOUT_MAX | ucon::RX_TIMEOUT_EN;
                self.write_reg(reg::UCON, ucon_value);

                // Acknowledge stale interrupts, then unmask RX only.
                self.write_reg(reg::UINTP, uint::ALL);
                self.write_reg(reg::UINTM, uint::ALL & !uint::RXD);
            }
            IrqStyle::AppleS5l => {
                self.write_reg(reg::UTRSTAT, utrstat::APPLE_ALL_FLAGS);
                ucon_value |=
                    ucon::APPLE_RXTHRESH_EN | ucon::APPLE_RXTO_EN | ucon::APPLE_RXTO_LEGACY_EN;
                self.write_reg(reg::UCON, ucon_value);
            }
        }
    }

    pub fn receive(&mut self, token: &mut CleanLockToken) {
        let mut received = false;
        let fifo_mode = self.read_reg(reg::UFCON) & ufcon::FIFO_EN != 0;

        // The Apple flags are acknowledged before draining so that
        // characters arriving mid-drain raise them again.
        if self.variant.irq == IrqStyle::AppleS5l {
            self.write_reg(reg::UTRSTAT, utrstat::APPLE_RX_FLAGS);
        }

        for _ in 0..RX_DRAIN_LIMIT {
            let pending = if fifo_mode {
                self.read_reg(reg::UFSTAT) & self.variant.fifo.rx_pending_mask() != 0
            } else {
                self.read_reg(reg::UTRSTAT) & utrstat::RX_READY != 0
            };
            if !pending {
                break;
            }

            // The error status applies to the next character read.
            let uerstat_value = self.read_reg(reg::UERSTAT);
            let c = self.read_data();
            if uerstat_value & uerstat::ANY == 0 && c != 0 {
                debug_input(c, token);
                received = true;
            }
        }

        if self.variant.irq == IrqStyle::S3c6400 {
            self.write_reg(reg::UINTP, uint::RXD);
        }

        if received {
            debug_notify(token);
        }
    }

    pub fn send(&mut self, data: u8) {
        if self.read_reg(reg::UFCON) & ufcon::FIFO_EN != 0 {
            while self.read_reg(reg::UFSTAT) & self.variant.fifo.tx_full_mask() != 0 {}
        } else {
            while self.read_reg(reg::UTRSTAT) & utrstat::TX_EMPTY == 0 {}
        }
        self.write_data(data);
    }

    pub fn write(&mut self, buf: &[u8]) {
        // An unconfigured port never drains its FIFO, so send() would
        // spin forever.
        if self.read_reg(reg::UCON) & ucon::MODE_MASK == 0 {
            return;
        }

        for &b in buf {
            match b {
                8 | 0x7F => {
                    self.send(8);
                    self.send(b' ');
                    self.send(8);
                }
                b'\n' => {
                    self.send(b'\r');
                    self.send(b'\n');
                }
                _ => self.send(b),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{calculate_baud_registers, is_compatible, DivisorStyle, Variant};

    #[test]
    fn recognizes_supported_compatibles() {
        assert!(is_compatible("samsung,s3c6400-uart"));
        assert!(is_compatible("samsung,exynos4210-uart"));
        assert!(is_compatible("apple,s5l-uart"));
        assert!(is_compatible("google,gs101-uart"));
        assert!(is_compatible(
            "samsung,exynos7885-uart\0samsung,exynos850-uart"
        ));
        assert!(!is_compatible("samsung,s3c2410-uart"));
        assert!(!is_compatible("samsung,exynos-gpio"));
    }

    #[test]
    fn specific_compatible_selects_its_variant() {
        assert_eq!(
            Variant::from_compatible("google,gs101-uart"),
            Some(Variant::GS101)
        );
        assert_eq!(
            Variant::from_compatible("samsung,exynos7885-uart\0samsung,exynos850-uart"),
            Some(Variant::EXYNOS)
        );
        assert_eq!(Variant::from_compatible("samsung,exynos7885-uart"), None);
    }

    #[test]
    fn integer_divisor_rounds_to_nearest() {
        // Apple S5L: 24 MHz reference at 115200 baud.
        assert_eq!(
            calculate_baud_registers(24_000_000, 115_200, DivisorStyle::Integer),
            Some((12, None))
        );
        assert_eq!(
            calculate_baud_registers(24_000_000, 0, DivisorStyle::Integer),
            None
        );
    }

    #[test]
    fn fractional_divisor_splits_sixteenths() {
        // 100 MHz / 115200 = 868.05: whole part 54 (UBRDIV 53),
        // remainder 4 sixteenths.
        assert_eq!(
            calculate_baud_registers(100_000_000, 115_200, DivisorStyle::Fractional),
            Some((53, Some(4)))
        );
        assert_eq!(
            calculate_baud_registers(100_000_000, 115_200, DivisorStyle::Table),
            Some((53, Some(0x2222)))
        );
        // Divisors below one whole step cannot be represented.
        assert_eq!(
            calculate_baud_registers(100_000, 115_200, DivisorStyle::Fractional),
            None
        );
    }

    #[test]
    fn reg_io_width_overrides_variant_default() {
        assert!(Variant::APPLE_S5L.wide_data);
        assert!(Variant::EXYNOS.with_reg_io_width(Some(4)).wide_data);
        assert!(!Variant::GS101.with_reg_io_width(Some(1)).wide_data);
        assert!(!Variant::EXYNOS.with_reg_io_width(None).wide_data);
    }
}
