import 'dart:async';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:material_ui/material_ui.dart';

/// Pixel-exact preview of the handlebar screen: the same 128x64 frame that
/// is sent to the Main Module. When [flash] is set it inverts 4 times a
/// second, like the Front Module does.
class LcdPreview extends StatefulWidget {
  final OledFrame frame;
  final bool flash;
  final double? width;
  final Color pixelColour;
  const LcdPreview({super.key, required this.frame, this.flash = false, this.width, this.pixelColour = const Color(0xFF5ABEFF)});

  @override
  State<LcdPreview> createState() => _LcdPreviewState();
}

class _LcdPreviewState extends State<LcdPreview> {
  Timer? _timer;
  bool _inverted = false;

  @override
  void initState() {
    super.initState();
    _sync();
  }

  @override
  void didUpdateWidget(LcdPreview old) {
    super.didUpdateWidget(old);
    _sync();
  }

  void _sync() {
    if (widget.flash && _timer == null) {
      _timer = Timer.periodic(const Duration(milliseconds: 125), (_) => setState(() => _inverted = !_inverted));
    } else if (!widget.flash && _timer != null) {
      _timer!.cancel();
      _timer = null;
      _inverted = false;
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: (context, c) {
      final w = widget.width ?? c.maxWidth;
      return Semantics(
        label: 'Handlebar screen preview',
        child: Container(
          width: w,
          padding: EdgeInsets.all(w * 0.035),
          decoration: BoxDecoration(
            color: const Color(0xFF05070A),
            borderRadius: BorderRadius.circular(w * 0.045),
            border: Border.all(color: const Color(0xFF263241), width: 2),
            boxShadow: [BoxShadow(color: widget.pixelColour.withValues(alpha: 0.10), blurRadius: 30)],
          ),
          child: AspectRatio(
            aspectRatio: 2,
            child: CustomPaint(painter: _LcdPainter(widget.frame, _inverted, widget.pixelColour)),
          ),
        ),
      );
    });
  }
}

class _LcdPainter extends CustomPainter {
  final OledFrame frame;
  final bool inverted;
  final Color colour;
  _LcdPainter(this.frame, this.inverted, this.colour);

  @override
  void paint(Canvas canvas, Size size) {
    final px = size.width / OledFrame.width;
    final on = Paint()..color = colour;
    final off = Paint()..color = colour.withValues(alpha: 0.05);
    for (var y = 0; y < OledFrame.height; y++) {
      for (var x = 0; x < OledFrame.width; x++) {
        final lit = frame.pixel(x, y) != inverted;
        canvas.drawRect(Rect.fromLTWH(x * px, y * px, px * 0.86, px * 0.86), lit ? on : off);
      }
    }
  }

  @override
  bool shouldRepaint(_LcdPainter old) => old.frame != frame || old.inverted != inverted || old.colour != colour;
}
