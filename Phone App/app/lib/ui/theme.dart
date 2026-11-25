import 'package:material_ui/material_ui.dart';

/// Colours used across the app (dark, high contrast, demo-friendly).
abstract final class Palette {
  static const background = Color(0xFF080B10);
  static const surface = Color(0xFF111821);
  static const surfaceHigh = Color(0xFF18212D);
  static const outline = Color(0xFF243142);
  static const text = Color(0xFFEAF1F8);
  static const muted = Color(0xFF8A99AB);
  static const accent = Color(0xFF3CD3FF); // navigation / links
  static const danger = Color(0xFFFF4D61); // close pass
  static const warning = Color(0xFFFFB224); // approaching vehicle
  static const ok = Color(0xFF3DDC97);
}

ThemeData buildTheme() {
  final scheme = ColorScheme.fromSeed(
    seedColor: Palette.accent,
    brightness: Brightness.dark,
  ).copyWith(
    primary: Palette.accent,
    onPrimary: const Color(0xFF00141C),
    error: Palette.danger,
    surface: Palette.surface,
    onSurface: Palette.text,
    surfaceContainerHighest: Palette.surfaceHigh,
    outline: Palette.outline,
  );
  return ThemeData(
    fontFamily: 'Roboto',
    colorScheme: scheme,
    scaffoldBackgroundColor: Palette.background,
    appBarTheme: const AppBarTheme(
      backgroundColor: Palette.background,
      foregroundColor: Palette.text,
      elevation: 0,
      scrolledUnderElevation: 0,
      centerTitle: false,
    ),
    cardTheme: CardThemeData(
      color: Palette.surface,
      elevation: 0,
      margin: EdgeInsets.zero,
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(22),
        side: const BorderSide(color: Palette.outline),
      ),
    ),
    filledButtonTheme: FilledButtonThemeData(
      style: FilledButton.styleFrom(
        minimumSize: const Size(0, 54),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        textStyle: const TextStyle(fontFamily: 'Roboto', fontSize: 16, fontWeight: FontWeight.w600),
      ),
    ),
    outlinedButtonTheme: OutlinedButtonThemeData(
      style: OutlinedButton.styleFrom(
        minimumSize: const Size(0, 50),
        side: const BorderSide(color: Palette.outline),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      ),
    ),
    inputDecorationTheme: InputDecorationTheme(
      filled: true,
      fillColor: Palette.surfaceHigh,
      border: OutlineInputBorder(borderRadius: BorderRadius.circular(16), borderSide: BorderSide.none),
      contentPadding: const EdgeInsets.symmetric(horizontal: 18, vertical: 16),
    ),
    snackBarTheme: const SnackBarThemeData(behavior: SnackBarBehavior.floating),
  );
}

/// Small grey section label.
class SectionLabel extends StatelessWidget {
  final String text;
  const SectionLabel(this.text, {super.key});

  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.only(left: 4, bottom: 8),
        child: Text(text.toUpperCase(),
            style: const TextStyle(color: Palette.muted, fontSize: 12, fontWeight: FontWeight.w700, letterSpacing: 1.2)),
      );
}
