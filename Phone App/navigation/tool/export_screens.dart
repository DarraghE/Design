// Writes many handlebar screens two ways, as test vectors for the Front
// Module firmware (which must decode exactly what the phone sends):
//   <dir>/packets.txt  "name packet1hex packet2hex ..." per line: the display
//                      packets the phone sends for that screen (see
//                      lib/src/display_codec.dart for the format)
//   <dir>/dart.txt     each screen as text art ('#' = lit pixel)
// Run: dart run tool/export_screens.dart [out_dir]
import 'dart:io';

import 'package:bike_navigation/bike_navigation.dart';

Manoeuvre _m(ManoeuvreKind k, [int? exit]) =>
    Manoeuvre(kind: k, instruction: '', pointIndex: 0, alongM: 0, exitNumber: exit);

NavState _state(Manoeuvre next, double distance,
    {Manoeuvre? then, bool show = true, bool flash = false}) {
  return NavState(
    next: next,
    then: show ? then : null,
    distanceToNextM: distance,
    showTurn: show,
    symbols: show
        ? [...displaySequence(next), if (then != null) ...displaySequence(then)]
        : const [LedSymbol.straight],
    offRoute: false,
    arrived: false,
    flash: flash,
    alongM: 0,
    offsetM: 0,
  );
}

void main(List<String> args) {
  final dir = Directory(args.isEmpty ? 'screens_out' : args.first)..createSync(recursive: true);
  final screens = <String, (OledFrame, bool)>{};
  const distances = <double>[0, 3, 4.6, 5, 47, 99, 99.5, 100, 104, 105, 123, 994, 995, 996, 2340, 2350, 9949, 9950, 12400, 65000];
  for (final k in ManoeuvreKind.values) {
    for (final d in distances) {
      screens['${k.name}_${d}m'] = (renderOled(_state(_m(k), d)), false);
      screens['${k.name}_${d}m_hidden'] = (renderOled(_state(_m(k), d, show: false)), false);
    }
  }
  for (var exit = 1; exit <= 9; exit++) {
    screens['roundabout_exit$exit'] = (renderOled(_state(_m(ManoeuvreKind.roundabout, exit), 80)), false);
  }
  final flashing = _state(_m(ManoeuvreKind.right), 4, flash: true);
  screens['flashing_right'] = (renderOled(flashing), flashing.flash);
  screens['arrived'] = (renderOled(NavState(next: null, distanceToNextM: 0, showTurn: false, symbols: const [LedSymbol.arrive], offRoute: false, arrived: true, alongM: 0, offsetM: 0)), false);
  screens['rerouting'] = (renderOled(NavState(next: _m(ManoeuvreKind.left), distanceToNextM: 50, showTurn: true, symbols: const [LedSymbol.rerouting], offRoute: true, arrived: false, alongM: 0, offsetM: 0)), false);
  screens['close_pass_alert'] = (renderCloseAlert(distanceM: 0.8, speedKmh: 42), true);
  screens['idle'] = (renderIdle(), false);
  screens['blank'] = (OledFrame(), false);

  final packets = StringBuffer(), art = StringBuffer();
  var frameNumber = 0, total = 0, largest = 0;
  for (final e in screens.entries) {
    final (frame, flash) = e.value;
    final encoded = encodeDisplayPackets(frame, frameNumber: frameNumber++ & 0xFF, flash: flash);
    total += encoded.compressedBytes;
    if (encoded.compressedBytes > largest) largest = encoded.compressedBytes;
    final hex = encoded.packets
        .map((p) => p.map((b) => b.toRadixString(16).padLeft(2, '0')).join())
        .join(' ');
    packets.writeln('${e.key} $hex');
    art.writeln('== ${e.key}${flash ? ' (flash)' : ''}\n${frame.toAscii()}');
  }
  File('${dir.path}/packets.txt').writeAsStringSync(packets.toString());
  File('${dir.path}/dart.txt').writeAsStringSync(art.toString());
  print('${screens.length} screens written to ${dir.path}. '
      'Compressed size: average ${total ~/ screens.length} bytes, largest $largest bytes (raw 1024).');
}
