/// Cycling navigation for the phone app.
///
/// Pure Dart (no Flutter imports) so it can be unit tested and replayed on a
/// laptop. The Flutter app feeds it GPS fixes, draws each handlebar screen
/// with [renderOled] and sends it, compressed by [encodeDisplayPackets], to
/// the Main Module, which passes it on to the Front Module.
library;

export 'src/geo.dart';
export 'src/route.dart';
export 'src/ors_client.dart';
export 'src/symbols.dart';
export 'src/oled.dart';
export 'src/display_codec.dart';
export 'src/engine.dart';
export 'src/session.dart';
export 'src/junctions.dart';
export 'src/replay.dart';
