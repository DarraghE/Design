import 'dart:io';

import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../closepass/incident.dart';
import 'incident_screen.dart';
import 'theme.dart';

/// Every recorded close pass, newest first.
class RecordingsScreen extends StatelessWidget {
  const RecordingsScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return Scaffold(
      appBar: AppBar(title: const Text('Recordings', style: TextStyle(fontWeight: FontWeight.w800))),
      body: ListenableBuilder(
        listenable: app.store,
        builder: (context, _) {
          final items = app.store.incidents;
          if (items.isEmpty) {
            return const Center(
              child: Padding(
                padding: EdgeInsets.all(40),
                child: Column(mainAxisSize: MainAxisSize.min, children: [
                  Icon(Icons.shield_outlined, size: 56, color: Palette.muted),
                  SizedBox(height: 14),
                  Text('No close passes yet', style: TextStyle(fontSize: 18, fontWeight: FontWeight.w700)),
                  SizedBox(height: 6),
                  Text('When the Main Module detects one, its photos and LiDAR replay appear here.',
                      textAlign: TextAlign.center, style: TextStyle(color: Palette.muted)),
                ]),
              ),
            );
          }
          return ListView.separated(
            padding: const EdgeInsets.fromLTRB(16, 4, 16, 24),
            itemCount: items.length,
            separatorBuilder: (_, _) => const SizedBox(height: 10),
            itemBuilder: (context, n) => _IncidentTile(incident: items[n]),
          );
        },
      ),
    );
  }
}

class _IncidentTile extends StatelessWidget {
  final Incident incident;
  const _IncidentTile({required this.incident});

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    final i = incident;
    final best = i.bestPhotoIndex;
    final file = best >= 0 ? app.store.photoFile(i, best) : null;
    return Card(
      clipBehavior: Clip.antiAlias,
      child: InkWell(
        onTap: () => Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => IncidentScreen(incidentId: i.id))),
        child: Row(children: [
          SizedBox(
            width: 112,
            height: 92,
            child: file != null && file.existsSync()
                ? Image.file(File(file.path), fit: BoxFit.cover, gaplessPlayback: true)
                : const ColoredBox(color: Palette.surfaceHigh, child: Icon(Icons.image_not_supported_outlined, color: Palette.muted)),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Row(children: [
                  Expanded(child: PlateText(incident: i, size: 18)),
                  UploadIcon(status: i.uploadStatus),
                ]),
                const SizedBox(height: 4),
                Text(
                  [
                    if (i.distanceM != null) '${i.distanceM!.toStringAsFixed(2)} m',
                    if (i.speedKmh != null) '${i.speedKmh!.round()} km/h',
                  ].join('  ·  '),
                  style: const TextStyle(color: Palette.danger, fontWeight: FontWeight.w700),
                ),
                const SizedBox(height: 2),
                Text('${formatWhen(i.time)}${i.simulated ? '  ·  simulated' : ''}',
                    style: const TextStyle(color: Palette.muted, fontSize: 13)),
              ]),
            ),
          ),
        ]),
      ),
    );
  }
}

/// The plate, or what is happening with it.
class PlateText extends StatelessWidget {
  final Incident incident;
  final double size;
  const PlateText({super.key, required this.incident, this.size = 18});

  @override
  Widget build(BuildContext context) {
    final p = incident.plate;
    final (text, colour) = switch (incident.plateStatus) {
      PlateStatus.found || PlateStatus.uncertain when p != null && p.found => (p.plate, Palette.text),
      PlateStatus.reading || PlateStatus.waiting => ('Reading plate…', Palette.muted),
      PlateStatus.noKey => ('Plate not read', Palette.muted),
      PlateStatus.failed => ('Plate read failed', Palette.warning),
      _ => ('No plate found', Palette.muted),
    };
    return Text(text,
        maxLines: 1,
        overflow: TextOverflow.ellipsis,
        style: TextStyle(fontSize: size, fontWeight: FontWeight.w800, color: colour, letterSpacing: p?.found == true ? 1.2 : 0));
  }
}

class UploadIcon extends StatelessWidget {
  final UploadStatus status;
  const UploadIcon({super.key, required this.status});

  @override
  Widget build(BuildContext context) => switch (status) {
        UploadStatus.uploaded => const Tooltip(message: 'Uploaded', child: Icon(Icons.cloud_done, color: Palette.ok, size: 20)),
        UploadStatus.uploading => const SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2)),
        UploadStatus.failed => const Tooltip(message: 'Upload failed', child: Icon(Icons.cloud_off, color: Palette.warning, size: 20)),
        UploadStatus.waiting || UploadStatus.notConfigured =>
          const Tooltip(message: 'Not uploaded yet', child: Icon(Icons.cloud_upload_outlined, color: Palette.muted, size: 20)),
        UploadStatus.notNeeded => const SizedBox.shrink(),
      };
}

String formatWhen(DateTime t) {
  const months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'];
  String two(int v) => v.toString().padLeft(2, '0');
  return '${t.day} ${months[t.month - 1]} ${t.year}, ${two(t.hour)}:${two(t.minute)}:${two(t.second)}';
}
