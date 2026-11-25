import 'package:material_ui/material_ui.dart';

import 'app.dart';
import 'ui/theme.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const _Boot());
}

/// Loads settings and recordings, then shows the app.
class _Boot extends StatefulWidget {
  const _Boot();

  @override
  State<_Boot> createState() => _BootState();
}

class _BootState extends State<_Boot> {
  late final Future<AppServices> _services = AppServices.create();

  @override
  Widget build(BuildContext context) => FutureBuilder<AppServices>(
        future: _services,
        builder: (context, snap) {
          if (snap.hasData) return BikeNavApp(services: snap.data!);
          return MaterialApp(
            debugShowCheckedModeBanner: false,
            theme: buildTheme(),
            home: Scaffold(
              body: Center(
                child: snap.hasError
                    ? Padding(padding: const EdgeInsets.all(24), child: Text('Could not start: ${snap.error}'))
                    : const CircularProgressIndicator(),
              ),
            ),
          );
        },
      );
}
